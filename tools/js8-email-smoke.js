#!/usr/bin/env node
"use strict";

const assert=require("assert");
const fs=require("fs");
const Email=require("../data/js8-email.js");
const Protocol=require("../data/js8-protocol.js");

const aprs=Email.normalizeGateway({id:"aprs",name:"APRS EMAIL-2",target:"@APRSIS",
  dialFrequencyHz:7078000,offsetHz:1500,format:"aprs-email2",maxBodyLength:40,
  characterPolicy:"aprs"});
const draft=Email.buildDraft(aprs,"user@example.com","Příjezd v 18:00");
assert.strictEqual(draft.payload,
  "@APRSIS CMD :EMAIL-2  :USER@EXAMPLE.COM PRIJEZD V 1800");
assert.strictEqual(Email.getBodyBudget(aprs,"x".repeat(60)+"@x.cz"),1);
assert.throws(()=>Email.buildDraft(aprs,"not-an-email","TEST"),/valid email/);
assert.throws(()=>Email.buildDraft(aprs,"user@example.com","X".repeat(41)),/at most 40/);

const template=Email.normalizeGateway({id:"custom",name:"Custom",target:"OK1XYZ",
  dialFrequencyHz:7078000,offsetHz:1200,format:"template",
  template:"{TARGET} MSG EMAIL {EMAIL} {BODY}",maxBodyLength:60,characterPolicy:"js8"});
assert.strictEqual(Email.buildDraft(template,"a@b.cz","test").payload,
  "OK1XYZ MSG EMAIL A@B.CZ TEST");
assert.throws(()=>Email.normalizeGateway({...template,template:"{TARGET} {BODY}"}),
  /\{EMAIL\}/);
assert.strictEqual(Email.normalizeTarget("ok1aaa>ok2bbb"),"OK1AAA>OK2BBB");

const transport=Email.transportParts(draft.payload,aprs.target);
const frames=Protocol.buildReplyFrames({myCall:"OK1HRA",toCall:transport.toCall,
  text:transport.text});
assert(frames.length>1);
assert(frames.some(frame=>(frame.frameType&4)===0));
const dictionary=new Protocol.JscDictionary(fs.readFileSync(
  require("path").join(__dirname,"../data/js8-jsc.bin")));
assert.strictEqual(Protocol.checksum16("123456789"),"54G");
const decoded=frames.slice(1).map(frame=>Protocol.decodeFrame({...frame,submode:0,
  offsetHz:1500,slotUtcMs:0},dictionary).text).join("");
assert(decoded.includes("@EXAMPLE.COM"));
assert(decoded.includes(":EMAIL-2  :"));

const relayGateway=Email.normalizeGateway({...template,id:"relay",target:"OK1AAA>OK2BBB",
  format:"direct",template:undefined});
const relayDraft=Email.buildDraft(relayGateway,"a@b.cz","TEST");
assert.deepStrictEqual(Email.transportParts(relayDraft.payload,relayGateway.target),
  {toCall:"OK1AAA",text:">OK2BBB EMAIL A@B.CZ TEST"});

console.log(`JS8 EMAIL PASS payload=${draft.payload} frames=${frames.length}`);
