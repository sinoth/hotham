pub mod debug_data;
use debug_data::DebugData;

use std::io::{Error as IOError, ErrorKind};
use std::thread::{self, JoinHandle};
use tokio::net::{TcpListener, TcpStream};

use futures::join;
use futures_util::{StreamExt, TryStreamExt};
use tokio::runtime::Builder;
use tokio::sync::broadcast::{channel as broadcast_channel, Receiver, Sender};

use tokio_stream::wrappers::BroadcastStream;
use tokio_tungstenite::tungstenite;

use serde::{Deserialize, Serialize};
use serde_repr::*;
use std::fmt::Debug;

#[derive(Serialize_repr, Deserialize_repr, Clone, Debug)]
#[repr(u8)]
pub enum Command {
    Reset,
    Init,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct InitData {
    data: DebugData,
    session_id: u64,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub enum Message {
    Data(DebugData),
    Command(Command),
    Init(InitData),
    Error(String),
}

pub struct DebugServer {
    pub to_client: Sender<Message>,
    pub from_client: Receiver<Message>,
    session_id: u64,
    _handle: JoinHandle<()>,
}

async fn accept_connection(
    stream: TcpStream,
    to_hotham: Sender<Message>,
    from_hotham: Receiver<Message>,
) {
    let addr = stream
        .peer_addr()
        .expect("connected streams should have a peer address");

    let ws_stream = tokio_tungstenite::accept_async(stream)
        .await
        .expect("Error during the websocket handshake occurred");

    let (to_client, from_client) = ws_stream.split();

    let client_to_hotham = from_client
        .filter_map(|msg| async move {
            match msg {
                Ok(tungstenite::Message::Text(m)) => match serde_json::from_str::<Message>(&m) {
                    Ok(message) => Some(Ok(message)),
                    Err(e) => {
                        let error_message = format!("Error deserialising: {:?}", e);
                        eprintln!("{:?}", error_message);
                        Some(Ok(Message::Error(error_message)))
                    }
                },
                _ => None,
            }
        })
        .try_for_each(|v| futures::future::ready(to_hotham.send(v).map(|_| ()).map_err(|_| ())));

    let from_hotham = BroadcastStream::new(from_hotham).map(|message| match message {
        Ok(message) => {
            let json = serde_json::to_string(&message)
                .expect(&format!("Unable to deserialize {:?}", message));
            Ok(tungstenite::Message::Text(json))
        }
        Err(e) => Err(tokio_tungstenite::tungstenite::Error::Io(IOError::new(
            ErrorKind::Other,
            e.to_string(),
        ))),
    });
    let hotham_to_client = from_hotham.forward(to_client);

    let (r1, r2) = join!(client_to_hotham, hotham_to_client);
    r1.expect("Problem!");
    r2.expect("Problem 2!");
}

impl DebugServer {
    pub fn new() -> DebugServer {
        // These names are kind of confusing, so here's a little explanation:
        //
        // - to_client - use this to send a message from hotham to the websocket client
        // - from_client - use this to receive a message from the websocket client to hotham
        let (to_client, from_client) = broadcast_channel(16);
        let to_client_clone = to_client.clone();

        let handle = thread::spawn(move || {
            let rt = Builder::new_current_thread().enable_all().build().unwrap();
            rt.block_on(async {
                let addr = "127.0.0.1:8000".to_string();
                // Create the event loop and TCP listener we'll accept connections on.
                let try_socket = TcpListener::bind(&addr).await;
                let listener = try_socket.expect("Failed to bind");
                while let Ok((stream, _)) = listener.accept().await {
                    // - to_hotham - use this to send a message from the websocket client back to hotham
                    // - from_hotham - use this to receive a message from hotham back to the websocket client
                    let to_hotham = to_client_clone.clone();
                    let from_hotham = to_hotham.subscribe();

                    tokio::spawn(accept_connection(stream, to_hotham, from_hotham));
                }
            })
        });

        DebugServer {
            to_client,
            from_client,
            _handle: handle,
            session_id: rand::random(),
        }
    }

    pub fn sync(&mut self, debug_data_from_hotham: &DebugData) -> Option<DebugData> {
        let mut editable_data = None;
        let response: Option<Message> = match self.from_client.try_recv() {
            Ok(Message::Data(debug_data_from_client)) => {
                editable_data = Some(debug_data_from_client);
                Some(Message::Data(debug_data_from_hotham.clone()))
            }
            Ok(Message::Command(Command::Reset)) => {
                Some(Message::Data(debug_data_from_hotham.clone()))
            }
            Ok(Message::Command(Command::Init)) => Some(Message::Init(InitData {
                session_id: self.session_id,
                data: debug_data_from_hotham.clone(),
            })),
            Ok(error_message @ Message::Error(_)) => Some(error_message),
            Ok(_) => None,
            Err(_) => Some(Message::Data(debug_data_from_hotham.clone())),
        };

        if let Some(response) = response {
            self.to_client
                .send(response)
                .expect("Unable to update value");
            let _ = self.from_client.try_recv();
        }

        editable_data
    }
}

#[cfg(test)]
#[allow(unused_assignments)]
mod tests {
    #[derive(Deserialize, Serialize, Clone, Debug, Default)]
    struct Test {
        name: String,
    }

    #[derive(Deserialize, Serialize, Clone, Debug)]
    struct Info {
        count: usize,
    }

    use std::collections::HashMap;

    use tokio_tungstenite::{connect_async, tungstenite::protocol::Message as TungsteniteMessage};

    use crate::debug_data::{DebugEntity, DebugTransform};
    use futures_util::sink::SinkExt;

    use super::*;
    #[test]
    fn it_works() {
        // This is simulating the inside of Hotham.
        let mut server: DebugServer = DebugServer::new();
        let test_entity = DebugEntity {
            name: "Environment".to_string(),
            id: 0,
            transform: Some(DebugTransform {
                translation: [0., 0., 0.],
                rotation: [0., 0., 0.],
                scale: [1., 1., 1.],
            }),
            collider: None,
        };
        let mut entities = HashMap::new();
        entities.insert(0, test_entity.clone());

        let debug_data = DebugData { frame: 0, entities };

        let tokio_rt = Builder::new_current_thread().enable_all().build().unwrap();
        // Send an init message to the server..
        let mut stream = tokio_rt.block_on(async {
            let (socket, _) = connect_async("ws://127.0.0.1:8000").await.unwrap();
            let (mut write, read) = socket.split();
            let _ = write
                .send(TungsteniteMessage::Text(r#"{ "Command": 1 }"#.to_string()))
                .await;

            read
        });

        server.sync(&debug_data);

        let data = tokio_rt.block_on(async {
            let message = stream.next().await.unwrap().unwrap();

            // Note that we may not get an "init" here as the server might not have processed our message yet.
            // We cover both bases just to be sure.
            match message {
                TungsteniteMessage::Text(s) => match serde_json::from_str::<Message>(&s) {
                    Ok(Message::Data(d)) => d,
                    Ok(Message::Init(i)) => i.data,
                    _ => panic!("Unexpected message: {}", s),
                },
                _ => panic!("Unexpected message {:?}", message),
            }
        });

        assert_eq!(data.entities.get(&0).unwrap(), &test_entity);
    }
}
