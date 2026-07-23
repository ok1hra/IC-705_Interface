#!/usr/bin/env node
"use strict";

globalThis.crypto=require("crypto").webcrypto;
const assert=require("assert");
const F=require("../data/js8-file-transfer.js");

(async()=>{
  for(let length=0;length<=512;length+=1){
    const bytes=Uint8Array.from({length},(_,index)=>(index*73+length)&255);
    assert.deepStrictEqual(F.base32Decode(F.base32Encode(bytes)),bytes);
  }
  assert.strictEqual(F.crc16Ccitt(new TextEncoder().encode("123456789")),0x29b1);
  assert.strictEqual(F.hex(F.sha256Fallback(new TextEncoder().encode("abc"))),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  assert.strictEqual(F.sanitizeFileName("../Žluťoučký virus.exe"),"ZLUTOUCKY_VIRUS.EXE.BIN");
  assert.deepStrictEqual(F.parseRanges("01,04,09-0C"),[1,4,9,10,11,12]);
  assert.strictEqual(F.compactRanges([12,9,11,10,4,1]),"1,4,9-C");
  assert.throws(()=>F.enforceFileLimit(2049,F.PROFILES.NORMAL),/2048-byte/);
  assert.throws(()=>F.enforceFileLimit(1,F.PROFILES.JS8_60),/disabled/);

  const original=Uint8Array.from({length:512},(_,index)=>(index*19+7)&255);
  const prepared=await F.prepareBytes(original,{transferId:"K7M2QX",fileName:"map.png",mimeType:"image/png"});
  const offer=F.parseMessage(F.encodeOffer(prepared.manifest));
  assert.deepStrictEqual({...offer},{type:"offer",id:"K7M2QX",size:512,blockCount:16,
    blockSize:32,compression:"none",hash12:prepared.manifest.hash12,fileName:"MAP.PNG"});
  const control=[F.encodeAccept("K7M2QX",4,F.PROFILES.NORMAL),F.encodeEnd("K7M2QX",4),
    F.encodeAck("K7M2QX",4),F.encodeQuery("K7M2QX"),F.encodeComplete("K7M2QX",prepared.manifest.hash12),
    F.encodeCancel("K7M2QX","USER"),F.encodeReject("K7M2QX","BUSY")];
  assert.deepStrictEqual(control.map(text=>F.parseMessage(text).type),
    ["accept","end","ack","query","complete","cancel","reject"]);
  const firstData=F.parseMessage(F.encodeData(prepared.manifest.transferId,prepared.blocks[1]));
  assert.deepStrictEqual(F.decodeDataMessage(firstData,32),prepared.blocks[1].bytes);
  assert.throws(()=>F.decodeDataMessage({...firstData,crc:"0000"},32),/CRC/);
  assert.deepStrictEqual(F.parseMessage(F.encodeNacks("K7M2QX",[1,4,9,10,11,12])[0]).sequences,[1,4,9,10,11,12]);
  assert.strictEqual(F.parseMessage(F.encodeNacks("K7M2QX","ALL")[0]).sequences,"ALL");
  for(const invalid of ["~F1 D K7M2QX 1 0000", "~F1 A K7M2QX 0 Z",
    "~F1 C K7M2QX SHORT", "~F1 N K7M2QX 2/1 1"])
    assert.throws(()=>F.parseMessage(invalid));

  // Deterministic selective-repeat simulation: first pass loses 10% and
  // corrupts one block, duplicates are idempotent, NACK repairs only missing.
  const received=Array(prepared.blocks.length).fill(null),lost=new Set([2,7]),corrupt=5;
  for(const block of prepared.blocks){
    if(lost.has(block.sequence))continue;
    let wire=F.encodeData(prepared.manifest.transferId,block);
    if(block.sequence===corrupt)wire=wire.slice(0,-1)+(wire.endsWith("A")?"B":"A");
    const message=F.parseMessage(wire);
    try{received[message.sequence]=F.decodeDataMessage(message,block.binaryLength);}catch(_error){}
    if(block.sequence===3)received[message.sequence]=F.decodeDataMessage(F.parseMessage(F.encodeData(prepared.manifest.transferId,block)),block.binaryLength);
  }
  const missing=prepared.blocks.map(block=>block.sequence).filter(sequence=>!received[sequence]);
  assert.deepStrictEqual(missing,[2,5,7]);
  const nack=F.encodeNacks(prepared.manifest.transferId,missing)[0];
  assert.deepStrictEqual(F.parseMessage(nack).sequences,missing);
  for(const sequence of missing){const block=prepared.blocks[sequence];received[sequence]=F.decodeDataMessage(F.parseMessage(F.encodeData(prepared.manifest.transferId,block)),block.binaryLength);}
  const verified=await F.verifyReceived({blocks:received,blockCount:prepared.manifest.blockCount,
    originalSize:prepared.manifest.originalSize,hash12:prepared.manifest.hash12});
  assert.deepStrictEqual(verified.bytes,original);
  assert.strictEqual(verified.sha256Hex,prepared.manifest.sha256Hex);
  console.log(`JS8 FILE PASS bytes=${original.length} blocks=${prepared.blocks.length} repaired=${missing.join(",")}`);
})().catch(error=>{console.error(error);process.exitCode=1;});
