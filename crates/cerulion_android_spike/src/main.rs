// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-android-spike pub [--secs N] [--hz N] [--listen L]... [--connect C]... [--robot R]`
//! `cerulion-android-spike sub --connect C [--secs N] [--want N]`

use std::sync::atomic::AtomicBool;

use cerulion_android_spike::{run_pub, run_sub, Counters, PubOpts};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().map(String::as_str).unwrap_or("pub");
    let mut opts = PubOpts::default();
    let mut want: u64 = 10;
    let mut it = args.iter().skip(1);
    while let Some(a) = it.next() {
        let mut val = || it.next().cloned().unwrap_or_default();
        match a.as_str() {
            "--secs" => opts.secs = val().parse().unwrap_or(10),
            "--hz" => opts.hz = val().parse().unwrap_or(5),
            "--listen" => opts.listen.push(val()),
            "--connect" => opts.connect.push(val()),
            "--robot" => opts.robot = val(),
            "--want" => want = val().parse().unwrap_or(10),
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    println!(
        "spike: target_os={} arch={} mode={mode}",
        std::env::consts::OS,
        std::env::consts::ARCH
    );
    let result = match mode {
        "pub" => run_pub(&opts, &AtomicBool::new(false), &Counters::default()),
        "sub" => run_sub(&opts.connect, opts.secs, want).and_then(|n| {
            if n == 0 {
                Err("no frame arrived".to_string())
            } else {
                Ok(())
            }
        }),
        other => Err(format!("unknown mode: {other}")),
    };
    if let Err(e) = result {
        eprintln!("spike: FAILED {e}");
        std::process::exit(1);
    }
}
