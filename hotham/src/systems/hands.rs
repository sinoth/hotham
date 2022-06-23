use crate::{
    components::{hand::Handedness, AnimationController, Hand, RigidBody},
    gltf_loader::add_model_to_world,
    resources::{PhysicsContext, RenderContext, VulkanContext, XrContext},
    util::{is_space_valid, posef_to_isometry},
};
use hecs::{PreparedQuery, World};
use rapier3d::prelude::{ActiveCollisionTypes, ActiveEvents, ColliderBuilder, RigidBodyBuilder};
use nalgebra::Isometry3;

/// Hands system
/// Used to allow users to interact with objects using their controllers as representations of their hands
pub fn hands_system(
    query: &mut PreparedQuery<(&mut Hand, Option<&mut AnimationController>, &mut RigidBody)>,
    world: &mut World,
    xr_context: &XrContext,
    physics_context: &mut PhysicsContext,
) {
    for (_, (hand, animation_controller, rigid_body_component)) in query.query(world).iter() {
        // Get our the space and path of the hand.
        let time = xr_context.frame_state.predicted_display_time;
        let (space, path) = match hand.handedness {
            Handedness::Left => (
                &xr_context.left_hand_space,
                xr_context.left_hand_subaction_path,
            ),
            Handedness::Right =>
                match hand.aim_pose {
                    true => (
                        &xr_context.right_pointer_space,
                        xr_context.right_hand_subaction_path,
                    ),
                    false => (
                        &xr_context.right_hand_space,
                        xr_context.right_hand_subaction_path,
                    )
                },
        };

        // Locate the hand in the space.
        let space = space.locate(&xr_context.reference_space, time).unwrap();

        // Check it's valid before using it
        if !is_space_valid(&space) {
            return;
        }

        let pose = space.pose;

        let mut position = posef_to_isometry(pose);

        // get grip value
        let grip_value =
            openxr::ActionInput::get(&xr_context.grab_action, &xr_context.session, path)
                .unwrap()
                .current_state;
        
        let trigger_value =
            openxr::ActionInput::get(&xr_context.trigger_action, &xr_context.session, path)
                .unwrap()
                .current_state;

        // Print the offset if we are releasing grip
        if hand.grip_value > 0.0 && trigger_value <= 0.0 && !hand.debug_hand{
           //println!("[SIN] Offset: {}", hand.offset);
           println!("[SIN] Offset translation: {}", hand.offset.translation);
           println!("[SIN] Offset rotation: {}", hand.offset.rotation);
           println!("[SIN] Offset quaternion: ({}, {}, {}, {})",
                hand.offset.rotation.coords.x,
                hand.offset.rotation.coords.y,
                hand.offset.rotation.coords.z,
                hand.offset.rotation.coords.w);
           let euler = hand.offset.rotation.euler_angles();
           println!("[SIN] Offset angles (roll, pitch, yaw): ({}, {}, {})",
                euler.0 * 180.0 * std::f32::consts::PI,
                euler.1 * 180.0 * std::f32::consts::PI,
                euler.2 * 180.0 * std::f32::consts::PI);
        }

        // Apply to Hand
        hand.grip_value = trigger_value;

        // Apply to AnimationController
        if let Some(animation_controller) = animation_controller {
            animation_controller.blend_amount = grip_value;
        }

        if hand.needs_snapshot {
            hand.calibration_snapshot = position * hand.offset;
            hand.needs_snapshot = false;
        }

        if hand.calibration_mode {
            //hand.offset.translation.vector = position.translation.vector - hand.calibration_snapshot.translation.vector;
            //hand.offset.rotation = position.rotation * hand.calibration_snapshot.rotation;
            //position *= hand.calibration_snapshot.inverse();
            hand.offset = position.inverse() * hand.calibration_snapshot;
        }

        // apply transform
        let rigid_body = physics_context
            .rigid_bodies
            .get_mut(rigid_body_component.handle)
            .unwrap();

        position = position * hand.offset;
        rigid_body.set_next_kinematic_position(position);
        
    }
}

/// Convenience function to add a Hand and corresponding Mesh to the world
pub fn add_hand(
    models: &std::collections::HashMap<String, World>,
    model_name: &str,
    handedness: Handedness,
    world: &mut World,
    vulkan_context: &VulkanContext,
    render_context: &RenderContext,
    physics_context: &mut PhysicsContext,
    offset: Option<Isometry3<f32>>,
    debug_hand: bool,
    aim_pose: bool,
) {
    let hand = add_model_to_world(
        model_name,
        models,
        world,
        None,
        vulkan_context,
        &render_context.descriptor_set_layouts,
    )
    .unwrap();
    {
        // Add a hand component
        world
            .insert_one(
                hand,
                Hand::new(handedness),
            )
            .unwrap();

        if let Ok(mut hand_struct) = world.get_mut::<Hand>(hand) {
            hand_struct.debug_hand = debug_hand;
            hand_struct.aim_pose = aim_pose;
        }

        // Modify the animation controller
        if let Ok(mut animation_controller) = world.get_mut::<AnimationController>(hand) {
            animation_controller.blend_from = 0;
            animation_controller.blend_to = 1;
        }

        // Give it a collider and rigid-body
        let collider = ColliderBuilder::capsule_y(0.05, 0.02)
            .sensor(true)
            .active_collision_types(ActiveCollisionTypes::all())
            .active_events(ActiveEvents::CONTACT_EVENTS | ActiveEvents::INTERSECTION_EVENTS)
            .build();
        let rigid_body = RigidBodyBuilder::new_kinematic_position_based().build();
        let components = physics_context.get_rigid_body_and_collider(hand, rigid_body, collider);
        world.insert(hand, components).unwrap();
    }
}