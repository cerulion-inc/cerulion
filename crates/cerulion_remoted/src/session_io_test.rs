// SPDX-License-Identifier: AGPL-3.0-only

use super::*;
use tokio::io::{duplex, BufWriter};

enum DriverEvent {
    Network(SessionIoEvent),
}

impl From<SessionIoEvent> for DriverEvent {
    fn from(event: SessionIoEvent) -> Self {
        Self::Network(event)
    }
}

async fn within<T>(future: impl std::future::Future<Output = T>) -> T {
    tokio::time::timeout(Duration::from_secs(2), future)
        .await
        .expect("framing task must settle")
}

fn frame(body: &[u8]) -> Vec<u8> {
    let mut bytes = (body.len() as u32).to_be_bytes().to_vec();
    bytes.extend_from_slice(body);
    bytes
}

async fn receive(bytes: &[u8]) -> Vec<SessionIoEvent> {
    let (mut peer, reader) = duplex(bytes.len().max(1));
    peer.write_all(bytes).await.unwrap();
    drop(peer);
    let (events, mut receiver) = mpsc::channel::<DriverEvent>(8);
    within(read_client(reader, events)).await;
    let mut observed = Vec::new();
    while let Some(DriverEvent::Network(event)) = receiver.recv().await {
        observed.push(event);
    }
    observed
}

#[tokio::test]
async fn literal_client_frames_keep_their_order_and_meaning_on_repeated_runs() {
    let bytes = [
        frame(br#"{"type":"stdin","data":"AQID"}"#),
        frame(br#"{"type":"stdin_eof"}"#),
        frame(br#"{"type":"close"}"#),
    ]
    .concat();
    for _ in 0..2 {
        assert_eq!(
            receive(&bytes).await,
            vec![
                SessionIoEvent::Client(ClientFrame::Stdin {
                    data: "AQID".into()
                }),
                SessionIoEvent::Client(ClientFrame::StdinEof),
                SessionIoEvent::Client(ClientFrame::Close),
                SessionIoEvent::Failed(ExitReason::Disconnected),
            ]
        );
    }
}

#[tokio::test]
async fn oversized_and_invalid_frames_stop_before_a_following_valid_frame() {
    let valid = frame(br#"{"type":"close"}"#);
    let mut cases = vec![
        (MAX_SESSION_FRAME_BYTES as u32 + 1).to_be_bytes().to_vec(),
        u32::MAX.to_be_bytes().to_vec(),
        frame(b""),
        frame(b"{"),
        frame(br#"{"type":"unknown"}"#),
        frame(br#"{"type":"stdin","data":"%%%"}"#),
        frame(br#"{"type":"close","unexpected":true}"#),
        frame(br#"{"type":"resize","size":{"rows":0,"cols":80,"pixel_width":0,"pixel_height":0}}"#),
    ];
    for bytes in &mut cases {
        bytes.extend_from_slice(&valid);
        assert_eq!(
            receive(bytes).await,
            vec![SessionIoEvent::Failed(ExitReason::ProtocolError)]
        );
    }
}

#[tokio::test]
async fn exact_frame_cap_is_accepted_but_partial_prefix_and_body_are_disconnections() {
    let mut body = br#"{"type":"close"}"#.to_vec();
    body.resize(MAX_SESSION_FRAME_BYTES, b' ');
    assert_eq!(
        receive(&frame(&body)).await,
        vec![
            SessionIoEvent::Client(ClientFrame::Close),
            SessionIoEvent::Failed(ExitReason::Disconnected),
        ]
    );
    for bytes in [vec![], vec![0], vec![0, 0, 0], vec![0, 0, 0, 2, b'{']] {
        assert_eq!(
            receive(&bytes).await,
            vec![SessionIoEvent::Failed(ExitReason::Disconnected)]
        );
    }
}

#[tokio::test]
async fn a_closed_event_receiver_stops_the_owned_reader() {
    let bytes = frame(br#"{"type":"close"}"#);
    let (mut peer, reader) = duplex(bytes.len());
    peer.write_all(&bytes).await.unwrap();
    let (events, receiver) = mpsc::channel::<SessionIoEvent>(1);
    drop(receiver);
    within(read_client(reader, events)).await;
    assert!(peer.write_all(b"x").await.is_err());
}

#[tokio::test]
async fn cancellation_drops_the_partly_read_stream() {
    let (mut peer, reader) = duplex(1);
    let (events, _receiver) = mpsc::channel::<SessionIoEvent>(1);
    let task = tokio::spawn(read_client(reader, events));
    // Capacity one makes completion prove the reader consumed a prefix byte.
    within(peer.write_all(&[0, 0])).await.unwrap();
    task.abort();
    assert!(within(task).await.unwrap_err().is_cancelled());
    assert!(peer.write_all(b"x").await.is_err());
}

#[tokio::test]
async fn a_capacity_one_event_queue_preserves_delivery_order() {
    let bytes = [
        frame(br#"{"type":"stdin_eof"}"#),
        frame(br#"{"type":"close"}"#),
    ]
    .concat();
    let (mut peer, reader) = duplex(bytes.len());
    peer.write_all(&bytes).await.unwrap();
    drop(peer);
    let (events, mut receiver) = mpsc::channel::<SessionIoEvent>(1);
    let task = tokio::spawn(read_client(reader, events));
    for expected in [
        SessionIoEvent::Client(ClientFrame::StdinEof),
        SessionIoEvent::Client(ClientFrame::Close),
        SessionIoEvent::Failed(ExitReason::Disconnected),
    ] {
        assert_eq!(within(receiver.recv()).await, Some(expected));
    }
    within(task).await.unwrap();
    assert_eq!(receiver.recv().await, None);
}

#[tokio::test]
async fn server_frames_match_literal_wire_bytes_and_flush_before_completion() {
    for _ in 0..2 {
        let (writer, mut peer) = duplex(512);
        let (frames, receiver) = mpsc::channel(2);
        frames
            .send(ServerFrame::Started {
                session_id: "session".into(),
            })
            .await
            .unwrap();
        frames
            .send(ServerFrame::Error {
                message: "stopped".into(),
            })
            .await
            .unwrap();
        drop(frames);
        let (events, mut observations) = mpsc::channel::<SessionIoEvent>(1);
        let (done, completed) = oneshot::channel();
        within(write_server(
            writer,
            receiver,
            events,
            Duration::from_secs(1),
            done,
        ))
        .await;
        completed.await.unwrap();
        assert_eq!(observations.recv().await, None);
        let mut actual = Vec::new();
        peer.read_to_end(&mut actual).await.unwrap();
        assert_eq!(
            actual,
            [
                frame(br#"{"type":"started","session_id":"session"}"#),
                frame(br#"{"type":"error","message":"stopped"}"#),
            ]
            .concat()
        );
    }
}

async fn rejected_write<W: AsyncWrite + Unpin>(writer: W, frame: ServerFrame, timeout: Duration) {
    let (frames, receiver) = mpsc::channel(1);
    frames.send(frame).await.unwrap();
    drop(frames);
    let (events, mut observations) = mpsc::channel::<SessionIoEvent>(1);
    let (done, completed) = oneshot::channel();
    within(write_server(writer, receiver, events, timeout, done)).await;
    assert_eq!(
        observations.recv().await,
        Some(SessionIoEvent::Failed(ExitReason::Disconnected))
    );
    assert_eq!(observations.recv().await, None);
    assert!(completed.await.is_err());
}

fn started() -> ServerFrame {
    ServerFrame::Started {
        session_id: "session".into(),
    }
}

#[tokio::test]
async fn oversized_server_serialization_writes_nothing_and_never_completes() {
    let (writer, mut peer) = duplex(512);
    rejected_write(
        writer,
        ServerFrame::Error {
            message: "x".repeat(MAX_SESSION_FRAME_BYTES),
        },
        Duration::from_secs(1),
    )
    .await;
    let mut actual = Vec::new();
    peer.read_to_end(&mut actual).await.unwrap();
    assert!(actual.is_empty());
}

#[tokio::test]
async fn closed_peer_refuses_header_and_buffered_flush_without_completion() {
    let (writer, peer) = duplex(512);
    drop(peer);
    rejected_write(writer, started(), Duration::from_secs(1)).await;
    let (writer, peer) = duplex(512);
    drop(peer);
    // Both writes fit the real buffer; its flush reaches the closed peer.
    rejected_write(
        BufWriter::with_capacity(512, writer),
        started(),
        Duration::from_secs(1),
    )
    .await;
}

#[tokio::test]
async fn peer_closure_after_the_header_refuses_the_body_without_completion() {
    let (writer, mut peer) = duplex(4);
    let peer = tokio::spawn(async move {
        let mut header = [0; 4];
        peer.read_exact(&mut header).await.unwrap();
        assert_eq!(header, [0, 0, 0, 41]);
        drop(peer);
    });
    rejected_write(writer, started(), Duration::from_secs(1)).await;
    within(peer).await.unwrap();
}

#[tokio::test]
async fn a_stalled_write_reaches_its_deadline_without_completion() {
    let (writer, _peer) = duplex(1);
    rejected_write(writer, started(), Duration::from_millis(1)).await;
}
