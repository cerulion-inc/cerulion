use godot::prelude::*;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}

use godot::classes::Node3D;

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct rust_print_test {
    #[base]
    base: Base<Node3D>
}

use godot::classes::INode3D;
use core::time::Duration;
use iceoryx2::prelude::*;
use std::thread;

#[derive(Debug)]
#[repr(C)]
pub struct TransmissionData {
    pub x: i32,
    pub y: i32,
    pub funky: f64,
}

const CYCLE_TIME: Duration = Duration::from_secs(1);

pub fn subscribe_ice_oryx2(topic: String) -> Result<(), Box<dyn std::error::Error>> {
    godot_print!("Entered subscription method!");

    let node = NodeBuilder::new().create::<ipc::Service>()?;
    
    godot_print!("Created subscription node!");

    let service = node
        .service_builder(&"My/Funk/ServiceName".try_into()?)
        .publish_subscribe::<TransmissionData>()
        .open_or_create()?;

    godot_print!("Created subscription service!");

    let subscriber = service.subscriber_builder().create()?;

    godot_print!("About to subscribe!");

    while let NodeEvent::Tick = node.wait(CYCLE_TIME) {
        while let Some(sample) = subscriber.receive()? {
            godot_print!("received: {:?}", *sample);
            return Ok(());
        }
    }
    
    Ok(())
}

#[godot_api]
impl INode3D for rust_print_test {
    fn init(base: Base<Node3D>) -> Self {
        godot_print!("Hello, world!");
        rust_print_test {
            base
        }
    }

    fn ready(&mut self) {
        godot_print!("Before subscription!");
        let result = subscribe_ice_oryx2("My/Funk/ServiceName".to_string());
        // thread::spawn(|| {
        //         godot_print!("During subscription!");
        //         let result = subscribe_ice_oryx2("My/Funk/ServiceName".to_string());
        // });
        godot_print!("After subscription!");
    }

    fn process(&mut self, delta: f64) {
        // godot_print!("Processing!");
        // godot_print!("In Rust process loop! {}", delta);

    }
}