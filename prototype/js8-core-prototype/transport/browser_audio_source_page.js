// PROTOTYPE — native browser WebSocket reconnect and binary AUD1 acceptance.
await new Promise(done => {
  const output = document.querySelector("#result");
  const delivered = []; const epochs = []; const statuses = [];
  let wallJump = 0;
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const source = new window.Js8WsAudioSource.WsAudioSource(8000, {
    url:`${protocol}//${location.host}/audiows`, reconnectMs:20,
    wallNow:() => Date.now() + wallJump,
  }).onSamples((samples, rate, metadata) => {
    delivered.push({length:samples.length, rate,
      streamId:metadata.streamId, sequence:metadata.sequence,
      wireBytes:metadata.aud1Wire.byteLength,
      anchorUtcMs:metadata.anchorUtcMs});
    maybeFinish();
  }).onEpoch(epoch => {
    epochs.push(epoch);
    if (epochs.length === 2 && epoch.streamId === 202) wallJump = 600;
    maybeFinish();
  })
    .onStatus(status => statuses.push(status));

  let finished = false;
  function finish(pass, detail) {
    if (finished) return;
    finished = true;
    output.textContent = `BROWSER AUDIO SOURCE ${pass ? "PASS" : "FAIL"} ` +
      JSON.stringify(detail);
    document.body.dataset.pass = String(pass);
    source.stop();
    fetch("/result", {method:"POST", headers:{"Content-Type":"application/json"},
      body:JSON.stringify({pass, text:output.textContent})})
      .finally(done);
  }

  function maybeFinish() {
    if (finished || delivered.length < 15 || epochs.length < 3) return;
    const snapshot = source.state().timebase;
    const checks = {
      nativeWebSocket: source._session.socket instanceof WebSocket,
      threeMediaEpochs: epochs[0].streamId === 101 &&
        epochs[1].streamId === 202 && epochs[2].streamId === 202 &&
        epochs[2].reason === "clock-jump",
      exactDelivery: delivered.length === 15 && delivered.every(item =>
        item.length === 160 && item.rate === 8000 && item.wireBytes === 200),
      anchorsPresent: delivered.every(item => Number.isFinite(item.anchorUtcMs)),
      duplicateDropped: snapshot.transport.duplicatePackets === 1 &&
        statuses.some(status => status.type === "packet-dropped"),
      gapPreserved: snapshot.transport.sequenceGaps === 1 &&
        snapshot.transport.gapSamples === 160,
      reconnect: snapshot.transport.reconnects === 1 &&
        snapshot.media.streamId === 202,
      clockJump: snapshot.clock.jumps === 1 &&
        snapshot.clock.status === "unchecked" &&
        snapshot.media.reason === "clock-jump" &&
        snapshot.media.slotFlushes === 2,
    };
    finish(Object.values(checks).every(Boolean),
      {checks, delivered:delivered.length, epochs:epochs.length});
  }

  setTimeout(() => finish(false,
    {reason:"timeout", delivered:delivered.length, epochs:epochs.length,
      statuses}), 4000);
  source.start();
});
