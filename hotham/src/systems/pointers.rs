use ash::vk;
use egui::Pos2;
use hecs::{PreparedQuery, With, World};
use nalgebra::{
    point, vector, Isometry3, Orthographic3, Point3, Quaternion, Translation3, UnitQuaternion,
    Vector2,
};
use rapier3d::{
    math::Point,
    prelude::{InteractionGroups, Ray},
};

const POSITION_OFFSET: [f32; 3] = [0., 0.071173, -0.066082];

/* Original Precision
    -0.5581498959847122,
    0.8274912503663805,
    0.03413791007514528,
    -0.05061153302400824,
*/
const ROTATION_OFFSET: Quaternion<f32> =
    Quaternion::new(-0.558_149_8, 0.827_491_2, 0.034_137_9, -0.050_611_5);

use crate::{
    components::{hand::Handedness, pane::PaneInput, Info, Pane, Pointer, Transform, Visible},
    resources::{PhysicsContext, XrContext},
    util::{is_space_valid, posef_to_isometry},
};

/// Pointers system
/// Allows users to interact with `Pane`s using their controllers
pub fn pointers_system(
    query: &mut PreparedQuery<With<Visible, (&mut Pointer, &mut Transform)>>,
    world: &mut World,
    xr_context: &XrContext,
    physics_context: &mut PhysicsContext,
) {
    for (_, (pointer, transform)) in query.query(world).iter() {
        // Get our the space and path of the pointer.
        let time = xr_context.frame_state.predicted_display_time;
        let (space, path) = match pointer.handedness {
            Handedness::Left => (
                &xr_context.left_hand_space,
                xr_context.left_hand_subaction_path,
            ),
            Handedness::Right => (
                &xr_context.right_hand_space,
                xr_context.right_hand_subaction_path,
            ),
        };

        // Locate the pointer in the space.
        let space = space.locate(&xr_context.reference_space, time).unwrap();
        if !is_space_valid(&space) {
            return;
        }

        let pose = space.pose;

        // apply transform
        let mut position = posef_to_isometry(pose);
        apply_grip_offset(&mut position);

        transform.translation = position.translation.vector;
        transform.rotation = position.rotation;

        // get trigger value
        let trigger_value =
            openxr::ActionInput::get(&xr_context.trigger_action, &xr_context.session, path)
                .unwrap()
                .current_state;
        pointer.trigger_value = trigger_value;

        let ray_direction = transform.rotation.transform_vector(&vector![0., 1.0, 0.]);

        // Sweet baby ray
        let ray = Ray::new(Point::from(transform.translation), ray_direction);
        let max_toi = 40.0;
        let solid = true;
        let groups = InteractionGroups::new(0b10, 0b10);
        let filter = None;

        if let Some((handle, toi)) = physics_context.query_pipeline.cast_ray(
            &physics_context.colliders,
            &ray,
            max_toi,
            solid,
            groups,
            filter,
        ) {
            // The first collider hit has the handle `handle` and it hit after
            // the ray travelled a distance equal to `ray.dir * toi`.
            let hit_point = ray.point_at(toi); // Same as: `ray.origin + ray.dir * toi`
            let hit_collider = physics_context.colliders.get(handle).unwrap();
            let entity = unsafe { world.find_entity_from_id(hit_collider.user_data as _) };
            match world.get_mut::<Pane>(entity) {
                Ok(mut pane) => {
                    let pane_transform = hit_collider.position();
                    let cursor_location = get_cursor_location_for_pane(
                        &hit_point,
                        pane_transform,
                        &pane.resolution,
                        &pane.world_size,
                    );
                    pane.input = Some(PaneInput {
                        cursor_location,
                        trigger_value,
                    });
                }
                Err(_) => {
                    let info = world.get::<Info>(entity).map(|i| format!("{:?}", *i));
                    println!("[HOTHAM_POINTERS] Ray collided with object that does not have a pane: {:?} - {:?}", entity, info);
                }
            }
        }
    }
}

pub(crate) fn apply_grip_offset(position: &mut Isometry3<f32>) {
    let updated_rotation = position.rotation.quaternion() * ROTATION_OFFSET;
    let updated_translation = position.translation.vector
        - vector!(POSITION_OFFSET[0], POSITION_OFFSET[1], POSITION_OFFSET[2]);
    position.rotation = UnitQuaternion::from_quaternion(updated_rotation);
    position.translation = Translation3::from(updated_translation);
}

fn get_cursor_location_for_pane(
    hit_point: &Point3<f32>,
    pane_transform: &Isometry3<f32>,
    pane_extent: &vk::Extent2D,
    pane_world_size: &Vector2<f32>,
) -> Pos2 {
    let projected_hit_point = ray_to_pane_space(hit_point, pane_transform, pane_world_size);
    let transformed_hit_point = pane_transform
        .rotation
        .transform_point(&projected_hit_point);

    // Adjust the point such that 0,0 is the pane's top left
    let x = (transformed_hit_point.x + 1.) * 0.5;
    let y = ((transformed_hit_point.y * -1.) * 0.5) + 0.5;

    // Convert to screen coordinates
    let x_points = x * pane_extent.width as f32;
    let y_points = y * pane_extent.height as f32;

    Pos2::new(x_points, y_points)
}

fn ray_to_pane_space(
    hit_point: &Point3<f32>,
    pane_transform: &Isometry3<f32>,
    pane_world_size: &Vector2<f32>,
) -> Point3<f32> {
    // Translate the extents of the pane into world space, using the pane's translation.
    let (extent_x, extent_y) = (pane_world_size.x / 2., pane_world_size.y / 2.);
    let translated_extents = pane_transform * point![extent_x, extent_y, 0.];

    // Now build an orthographic matrix to project from world space into the pane's screen space
    let left = translated_extents.x - 1.;
    let right = translated_extents.x;
    let bottom = translated_extents.y - 1.;
    let top = translated_extents.y;
    let pane_projection = Orthographic3::new(left, right, bottom, top, 0., 1.);

    // Project the ray's hit point into pane space
    pane_projection.project_point(hit_point)
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;
    use ash::vk;

    #[cfg(target_os = "windows")]
    #[test]
    pub fn test_pointers_system() {
        use crate::{
            components::{Collider, Pane, Transform},
            resources::physics_context::{DEFAULT_COLLISION_GROUP, PANEL_COLLISION_GROUP},
            util::test_buffer,
        };
        use nalgebra::vector;
        use rapier3d::prelude::ColliderBuilder;

        let (mut xr_context, _) = XrContext::new().unwrap();
        let mut physics_context = PhysicsContext::default();
        let mut world = World::default();

        let pane = Pane {
            text: "Test Pane".to_string(),
            extent: vk::Extent2D {
                width: 300,
                height: 300,
            },
            framebuffer: vk::Framebuffer::null(),
            vertex_buffer: test_buffer(),
            index_buffer: test_buffer(),
            egui_context: Default::default(),
            raw_input: Default::default(),
            input: Default::default(),
            buttons: Vec::new(),
        };
        let pane_entity = world.spawn((pane,));

        // Place the pane *directly above* where the pointer will be located.
        let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.0)
            .sensor(true)
            .collision_groups(InteractionGroups::new(
                PANEL_COLLISION_GROUP,
                PANEL_COLLISION_GROUP,
            ))
            .translation(vector![-0.2, 2., -0.433918])
            .rotation(vector![(3. * std::f32::consts::PI) * 0.5, 0., 0.])
            .user_data(pane_entity.id() as _)
            .build();
        let handle = physics_context.colliders.insert(collider);
        let collider = Collider {
            collisions_this_frame: Vec::new(),
            handle,
        };
        world.insert_one(pane_entity, collider).unwrap();

        // Add a decoy collider to ensure we're using collision groups correctly.
        let collider = ColliderBuilder::cuboid(0.1, 0.1, 0.1)
            .sensor(true)
            .collision_groups(InteractionGroups::new(
                DEFAULT_COLLISION_GROUP,
                DEFAULT_COLLISION_GROUP,
            ))
            .translation(vector![-0.2, 1.5, -0.433918])
            .rotation(vector![(3. * std::f32::consts::PI) * 0.5, 0., 0.])
            .build();
        let handle = physics_context.colliders.insert(collider);
        let collider = Collider {
            collisions_this_frame: Vec::new(),
            handle,
        };
        world.spawn((collider,));

        let pointer_entity = world.spawn((
            Visible {},
            Pointer {
                handedness: Handedness::Left,
                trigger_value: 0.0,
            },
            Transform::default(),
        ));

        schedule(&mut physics_context, &mut world, &mut xr_context);

        let transform = world.get_mut::<Transform>(pointer_entity).unwrap();

        // Assert that the pointer has moved
        assert_relative_eq!(transform.translation, vector![-0.2, 1.328827, -0.433918]);

        let pane = world.get_mut::<Pane>(pane_entity).unwrap();
        let input = pane.input.clone().unwrap();
        assert_relative_eq!(input.cursor_location.x, 50.);
        assert_relative_eq!(input.cursor_location.y, 29.491043);
        assert_eq!(input.trigger_value, 0.);
    }

    #[cfg(target_os = "windows")]
    fn schedule(
        physics_context: &mut PhysicsContext,
        world: &mut hecs::World,
        xr_context: &mut XrContext,
    ) -> () {
        physics_context.update();
        pointers_system(&mut Default::default(), world, xr_context, physics_context)
    }

    #[test]
    pub fn test_get_cursor_location_for_pane() {
        let pane_transform = Isometry3::new(nalgebra::zero(), nalgebra::zero());
        let pane_extent = vk::Extent2D {
            width: 100,
            height: 100,
        };
        let pane_world_size = vector![1.0, 1.0];

        // Trivial example. Pane and hit point at origin:
        let result = get_cursor_location_for_pane(
            &point![0., 0., 0.],
            &pane_transform,
            &pane_extent,
            &pane_world_size,
        );
        assert_relative_eq!(result.x, 50.);
        assert_relative_eq!(result.y, 50.);

        // hit top left
        let result = get_cursor_location_for_pane(
            &point![-0.5, 0.5, 0.],
            &pane_transform,
            &pane_extent,
            &pane_world_size,
        );
        assert_relative_eq!(result.x, 0.);
        assert_relative_eq!(result.y, 0.);

        // hit top right
        let result = get_cursor_location_for_pane(
            &point![0.5, 0.5, 0.],
            &pane_transform,
            &pane_extent,
            &pane_world_size,
        );
        assert_relative_eq!(result.x, 100.);
        assert_relative_eq!(result.y, 0.);

        // hit bottom right
        let result = get_cursor_location_for_pane(
            &point![0.5, -0.5, 0.],
            &pane_transform,
            &pane_extent,
            &pane_world_size,
        );
        assert_relative_eq!(result.x, 100.);
        assert_relative_eq!(result.y, 100.);

        // hit bottom left
        let result = get_cursor_location_for_pane(
            &point![-0.5, -0.5, 0.],
            &pane_transform,
            &pane_extent,
            &pane_world_size,
        );
        assert_relative_eq!(result.x, 0.);
        assert_relative_eq!(result.y, 100.);
    }

    #[test]
    pub fn test_ray_to_pane_space() {
        let pane_transform = Isometry3::new(nalgebra::zero(), nalgebra::zero());
        let pane_world_size = vector![1.0, 1.0];

        let result = ray_to_pane_space(&point![0., 0., 0.], &pane_transform, &pane_world_size);
        assert_relative_eq!(result, point![0.0, 0.0, -1.0]);

        // hit top left
        let result = ray_to_pane_space(&point![-0.5, 0.5, 0.], &pane_transform, &pane_world_size);
        assert_relative_eq!(result, point![-1.0, 1.0, -1.0]);

        // hit top right
        let result = ray_to_pane_space(&point![0.5, 0.5, 0.], &pane_transform, &pane_world_size);
        assert_relative_eq!(result, point![1.0, 1.0, -1.0]);

        // hit bottom right
        let result = ray_to_pane_space(&point![0.5, -0.5, 0.], &pane_transform, &pane_world_size);
        assert_relative_eq!(result, point![1.0, -1.0, -1.0]);

        // hit bottom left
        let result = ray_to_pane_space(&point![-0.5, -0.5, 0.], &pane_transform, &pane_world_size);
        assert_relative_eq!(result, point![-1.0, -1.0, -1.0]);
    }
}
