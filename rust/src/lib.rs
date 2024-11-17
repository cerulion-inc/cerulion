use godot::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;
use std::thread;
use zenoh::*;
use zenoh::config::Config;
use tokio::runtime::Runtime;
use tokio::time::timeout;
use std::time::Duration;

#[derive(GodotClass)]
#[class(base=Node)]
struct ZenohSubscriber {
    base: Base<Node>,
    running: Arc<AtomicBool>,
    receiver: Option<Receiver<String>>,
    thread_handle: Option<thread::JoinHandle<()>>,
    #[export]
    topic: GString,
}

#[godot_api]
impl INode for ZenohSubscriber {
    fn init(base: Base<Node>) -> Self {
        godot_print!("Init");
        Self {
            base,
            running: Arc::new(AtomicBool::new(true)),
            receiver: None,
            thread_handle: None,
            topic: "example/topic".into(),
        }
    }

    fn ready(&mut self) {
        godot_print!("Ready");
        
        let running = self.running.clone();
        let (tx, rx) = channel();
        self.receiver = Some(rx);
        
        let topic = self.topic.to_string();
        
        let handle = thread::spawn(move || {
            match Runtime::new() {
                Ok(rt) => {
                    match rt.block_on(async {
                        let config = Config::default();
                        zenoh::open(config).await
                    }) {
                        Ok(session) => {
                            match rt.block_on(async {
                                session.declare_subscriber(topic).await
                            }) {
                                Ok(subscriber) => {
                                    while running.load(Ordering::SeqCst) {
                                        match rt.block_on(async {
                                            timeout(Duration::from_millis(100), subscriber.recv_async()).await
                                        }) {
                                            Ok(Ok(sample)) => {
                                                let _ = tx.send(format!("Received: {:?}", sample.payload().try_to_string().unwrap_or_else(|e| e.to_string().into())));
                                            }
                                            Ok(Err(e)) => godot_error!("Error receiving: {:?}", e),
                                            Err(_) => {
                                                // Timeout - check if we should continue running
                                                continue;
                                            }
                                        }
                                    }
                                }
                                Err(e) => godot_error!("Failed to create subscriber: {:?}", e),
                            }
                        }
                        Err(e) => godot_error!("Failed to open session: {:?}", e),
                    }
                }
                Err(e) => godot_error!("Failed to create runtime: {:?}", e),
            }
        });
        
        self.thread_handle = Some(handle);
        godot_print!("Thread spawned");
    }

    fn process(&mut self, _delta: f64) {
        if let Some(rx) = &self.receiver {
            while let Ok(msg) = rx.try_recv() {
                godot_print!("{}", msg);
            }
        }
    }

    fn exit_tree(&mut self) {
        godot_print!("Exiting");
        self.running.store(false, Ordering::SeqCst);
        
        if let Some(handle) = self.thread_handle.take() {
            match handle.join() {
                Ok(_) => godot_print!("Thread joined successfully"),
                Err(e) => godot_error!("Thread join error: {:?}", e),
            }
        }
    }
}

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}