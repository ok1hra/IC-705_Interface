// JS8Call ~F1 file-transfer protocol primitives and persistent session store.

(function(root,factory){
  const value=factory();
  if(typeof module==="object"&&module.exports)module.exports=value;
  else root.Js8FileTransfer=value;
})(typeof globalThis!=="undefined"?globalThis:self,function(){
  const BASE32="ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
  const PROTOCOL_PREFIX="~F1";
  const DB_NAME="ic705-js8-file-transfers";
  const DB_VERSION=1;
  const STORE_NAME="sessions";
  const DEFAULTS={protocolVersion:1,blockSizeBytes:32,maxContinuousTxSeconds:120,
    repairReserveRatio:.30,offerRetries:2,statusRetries:3,maxFileNameLength:24};
  const PROFILES={
    SLOW:{key:"SLOW",code:"S",label:"Slow",submode:4,periodSeconds:30,minBpm:5,maxBpm:10,hardLimit:1024,warningSize:512,windowSize:2,offerTimeoutSeconds:180,statusTimeoutSeconds:240},
    NORMAL:{key:"NORMAL",code:"N",label:"Normal",submode:0,periodSeconds:15,minBpm:10,maxBpm:20,hardLimit:2048,warningSize:1024,windowSize:4,offerTimeoutSeconds:120,statusTimeoutSeconds:150},
    FAST:{key:"FAST",code:"F",label:"Fast",submode:1,periodSeconds:10,minBpm:18,maxBpm:30,hardLimit:4096,warningSize:2048,windowSize:6,offerTimeoutSeconds:90,statusTimeoutSeconds:120},
    JS8_40:{key:"JS8_40",code:"4",label:"JS8 40",submode:2,periodSeconds:6,minBpm:30,maxBpm:50,hardLimit:8192,warningSize:4096,windowSize:8,offerTimeoutSeconds:60,statusTimeoutSeconds:90},
    JS8_60:{key:"JS8_60",code:"6",label:"JS8 60",submode:8,periodSeconds:4,minBpm:0,maxBpm:0,hardLimit:0,warningSize:0,windowSize:0,offerTimeoutSeconds:60,statusTimeoutSeconds:90}
  };
  const PROFILE_BY_CODE=Object.fromEntries(Object.values(PROFILES).map(item=>[item.code,item]));
  const PROFILE_BY_SUBMODE=Object.fromEntries(Object.values(PROFILES).map(item=>[item.submode,item]));

  function bytesOf(value){return value instanceof Uint8Array?value:new Uint8Array(value||0);}
  function toBase36(value){
    const number=Number(value);
    if(!Number.isSafeInteger(number)||number<0)throw new Error("Invalid Base36 number.");
    return number.toString(36).toUpperCase();
  }
  function fromBase36(value){
    const text=String(value||"").toUpperCase();
    if(!/^[0-9A-Z]+$/.test(text))throw new Error("Invalid Base36 number.");
    const number=parseInt(text,36);
    if(!Number.isSafeInteger(number))throw new Error("Base36 number is too large.");
    return number;
  }
  function base32Encode(input){
    const bytes=bytesOf(input);let output="",buffer=0,bits=0;
    for(const byte of bytes){buffer=(buffer<<8)|byte;bits+=8;while(bits>=5){bits-=5;output+=BASE32[(buffer>>>bits)&31];buffer&=(1<<bits)-1;}}
    if(bits)output+=BASE32[(buffer<<(5-bits))&31];
    return output;
  }
  function base32Decode(value){
    const text=String(value||"").trim().toUpperCase().replace(/=+$/,"");
    if(!/^[A-Z2-7]*$/.test(text))throw new Error("Invalid Base32 payload.");
    const out=[];let buffer=0,bits=0;
    for(const character of text){buffer=(buffer<<5)|BASE32.indexOf(character);bits+=5;if(bits>=8){bits-=8;out.push((buffer>>>bits)&255);buffer&=(1<<bits)-1;}}
    if(bits&&buffer!==0)throw new Error("Invalid Base32 trailing bits.");
    return Uint8Array.from(out);
  }
  function crc16Ccitt(input){
    let crc=0xffff;
    for(const byte of bytesOf(input)){crc^=byte<<8;for(let bit=0;bit<8;bit+=1)crc=(crc&0x8000)?((crc<<1)^0x1021)&0xffff:(crc<<1)&0xffff;}
    return crc;
  }
  function crcHex(input){return crc16Ccitt(input).toString(16).toUpperCase().padStart(4,"0");}
  function hex(input){return [...bytesOf(input)].map(value=>value.toString(16).padStart(2,"0")).join("");}
  function sha256Fallback(input){
    const source=bytesOf(input),bitLength=source.length*8,paddedLength=Math.ceil((source.length+9)/64)*64;
    const bytes=new Uint8Array(paddedLength);bytes.set(source);bytes[source.length]=0x80;
    const view=new DataView(bytes.buffer);view.setUint32(paddedLength-8,Math.floor(bitLength/0x100000000),false);view.setUint32(paddedLength-4,bitLength>>>0,false);
    const constants=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
    const hash=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19],words=new Uint32Array(64),rotate=(value,count)=>(value>>>count)|(value<<(32-count));
    for(let offset=0;offset<bytes.length;offset+=64){
      for(let index=0;index<16;index+=1)words[index]=view.getUint32(offset+index*4,false);
      for(let index=16;index<64;index+=1){const s0=rotate(words[index-15],7)^rotate(words[index-15],18)^(words[index-15]>>>3),s1=rotate(words[index-2],17)^rotate(words[index-2],19)^(words[index-2]>>>10);words[index]=(words[index-16]+s0+words[index-7]+s1)>>>0;}
      let [a,b,c,d,e,f,g,h]=hash;
      for(let index=0;index<64;index+=1){const sum1=rotate(e,6)^rotate(e,11)^rotate(e,25),choice=(e&f)^(~e&g),temp1=(h+sum1+choice+constants[index]+words[index])>>>0,sum0=rotate(a,2)^rotate(a,13)^rotate(a,22),majority=(a&b)^(a&c)^(b&c),temp2=(sum0+majority)>>>0;h=g;g=f;f=e;e=(d+temp1)>>>0;d=c;c=b;b=a;a=(temp1+temp2)>>>0;}
      hash[0]=(hash[0]+a)>>>0;hash[1]=(hash[1]+b)>>>0;hash[2]=(hash[2]+c)>>>0;hash[3]=(hash[3]+d)>>>0;hash[4]=(hash[4]+e)>>>0;hash[5]=(hash[5]+f)>>>0;hash[6]=(hash[6]+g)>>>0;hash[7]=(hash[7]+h)>>>0;
    }
    const output=new Uint8Array(32),outputView=new DataView(output.buffer);hash.forEach((value,index)=>outputView.setUint32(index*4,value,false));return output;
  }
  async function sha256(input){
    const subtle=globalThis.crypto&&globalThis.crypto.subtle;
    if(!subtle)return sha256Fallback(input);
    return new Uint8Array(await subtle.digest("SHA-256",bytesOf(input)));
  }
  function createTransferId(){
    const random=new Uint8Array(4);
    if(globalThis.crypto&&globalThis.crypto.getRandomValues)globalThis.crypto.getRandomValues(random);
    else for(let index=0;index<random.length;index+=1)random[index]=Math.floor(Math.random()*256);
    return base32Encode(random).slice(0,6);
  }
  function sanitizeFileName(value){
    let name=String(value||"FILE.BIN").split(/[\\/]/).pop().normalize("NFKD")
      .replace(/[\u0300-\u036f]/g,"").toUpperCase().replace(/[^A-Z0-9._-]/g,"_")
      .replace(/_+/g,"_").replace(/^\.+/,"").slice(0,DEFAULTS.maxFileNameLength);
    if(!name)name="FILE.BIN";
    if(/\.(?:EXE|COM|BAT|CMD|SCR|PS1|JS|VBS|APK)$/i.test(name))name=`${name.slice(0,20)}.BIN`;
    return name;
  }
  function normalizeCallsign(value){
    const call=String(value||"").trim().toUpperCase();
    if(!/^[A-Z0-9]{2,6}(?:\/P)?$/.test(call)||!/\d/.test(call))throw new Error("Enter a direct, packable peer callsign.");
    return call;
  }
  function profileForSubmode(submode){return PROFILE_BY_SUBMODE[Number(submode)]||PROFILES.NORMAL;}
  function estimateDuration(size,profile){
    if(!profile||!profile.minBpm||!profile.maxBpm)return null;
    return {optimisticMinutes:Math.ceil(size/profile.maxBpm),conservativeMinutes:Math.ceil(size/profile.minBpm),
      plannedMinutes:Math.ceil(size/profile.minBpm*(1+DEFAULTS.repairReserveRatio))};
  }
  function enforceFileLimit(size,profile){
    if(!Number.isInteger(size)||size<1)throw new Error("Select a non-empty file.");
    if(!profile||profile.hardLimit===0)throw new Error("JS8 60 file transfer is disabled.");
    if(size>profile.hardLimit)throw new Error(`File exceeds the ${profile.hardLimit}-byte ${profile.label} limit.`);
  }
  async function prepareBytes(input,options={}){
    const bytes=bytesOf(input),blockSize=Number(options.blockSize||DEFAULTS.blockSizeBytes);
    if(!Number.isInteger(blockSize)||blockSize<1||blockSize>255)throw new Error("Invalid block size.");
    const hashBytes=await sha256(bytes),blockCount=Math.ceil(bytes.length/blockSize);
    const blocks=[{sequence:0,binaryLength:hashBytes.length,crc16:crc16Ccitt(hashBytes),payloadBase32:base32Encode(hashBytes),bytes:hashBytes}];
    for(let index=0;index<blockCount;index+=1){
      const block=bytes.slice(index*blockSize,Math.min(bytes.length,(index+1)*blockSize));
      blocks.push({sequence:index+1,binaryLength:block.length,crc16:crc16Ccitt(block),payloadBase32:base32Encode(block),bytes:block});
    }
    const hashBase32=base32Encode(hashBytes);
    return {manifest:{protocol:1,transferId:options.transferId||createTransferId(),fileName:sanitizeFileName(options.fileName),
      mimeType:String(options.mimeType||"application/octet-stream").slice(0,80),originalSize:bytes.length,
      encodedSize:blocks.reduce((total,item)=>total+item.payloadBase32.length,0),compression:"none",
      blockSize,blockCount,sha256Hex:hex(hashBytes),hash12:hashBase32.slice(0,12)},blocks,bytes};
  }
  function encodeOffer(manifest){return `${PROTOCOL_PREFIX} O ${manifest.transferId} ${toBase36(manifest.originalSize)} ${toBase36(manifest.blockCount)} ${toBase36(manifest.blockSize)} ${manifest.compression==="none"?"N":"C"} ${manifest.hash12} ${sanitizeFileName(manifest.fileName)}`;}
  function encodeAccept(id,windowSize,profile){return `${PROTOCOL_PREFIX} A ${id} ${toBase36(windowSize)} ${profile.code||profile}`;}
  function encodeData(id,block){return `${PROTOCOL_PREFIX} D ${id} ${toBase36(block.sequence)} ${block.crc16.toString(16).toUpperCase().padStart(4,"0")} ${block.payloadBase32}`;}
  function encodeEnd(id,lastSequence){return `${PROTOCOL_PREFIX} E ${id} ${toBase36(lastSequence)}`;}
  function encodeAck(id,lastSequence){return `${PROTOCOL_PREFIX} K ${id} ${toBase36(lastSequence)}`;}
  function encodeQuery(id){return `${PROTOCOL_PREFIX} Q ${id}`;}
  function encodeComplete(id,hash12){return `${PROTOCOL_PREFIX} C ${id} ${hash12}`;}
  function encodeCancel(id,reason="USER"){return `${PROTOCOL_PREFIX} X ${id} ${reason}`;}
  function encodeReject(id,reason="POLICY"){return `${PROTOCOL_PREFIX} R ${id} ${reason}`;}
  function compactRanges(sequences){
    const sorted=[...new Set(sequences.map(Number).filter(Number.isSafeInteger))].sort((a,b)=>a-b),parts=[];
    for(let index=0;index<sorted.length;){let end=index;while(end+1<sorted.length&&sorted[end+1]===sorted[end]+1)end+=1;parts.push(end===index?toBase36(sorted[index]):`${toBase36(sorted[index])}-${toBase36(sorted[end])}`);index=end+1;}
    return parts.join(",");
  }
  function parseRanges(value){
    if(value==="ALL")return "ALL";
    if(!value)return [];
    const result=[];
    for(const part of value.split(",")){
      const match=/^([0-9A-Z]+)(?:-([0-9A-Z]+))?$/.exec(part);if(!match)throw new Error("Invalid NACK range.");
      const start=fromBase36(match[1]),end=match[2]?fromBase36(match[2]):start;
      if(end<start||end-start>4096)throw new Error("Invalid NACK range.");
      for(let value=start;value<=end;value+=1)result.push(value);
    }
    return [...new Set(result)];
  }
  function encodeNacks(id,sequences,maxLength=68){
    if(sequences==="ALL")return [`${PROTOCOL_PREFIX} N ${id} ALL`];
    const parts=compactRanges(sequences).split(","),chunks=[];let current="";
    for(const part of parts){const next=current?`${current},${part}`:part;if(next.length>maxLength&&current){chunks.push(current);current=part;}else current=next;}
    if(current)chunks.push(current);if(chunks.length<=1)return [`${PROTOCOL_PREFIX} N ${id} ${chunks[0]||"ALL"}`];
    return chunks.map((chunk,index)=>`${PROTOCOL_PREFIX} N ${id} ${index+1}/${chunks.length} ${chunk}`);
  }
  function parseMessage(input){
    const source=String(input||"").trim().toUpperCase(),at=source.indexOf(PROTOCOL_PREFIX);
    if(at<0)return null;
    const text=source.slice(at).replace(/\s+/g," "),fields=text.split(" ");
    if(fields[0]!==PROTOCOL_PREFIX||!fields[1]||!fields[2]||!/^[A-Z2-7]{6}$/.test(fields[2]))throw new Error("Invalid ~F1 message header.");
    const type=fields[1],id=fields[2];
    if(type==="O"){
      if(fields.length!==9||!/[NC]/.test(fields[6])||!/^[A-Z2-7]{12}$/.test(fields[7]))throw new Error("Invalid OFFER.");
      return {type:"offer",id,size:fromBase36(fields[3]),blockCount:fromBase36(fields[4]),blockSize:fromBase36(fields[5]),compression:fields[6]==="N"?"none":"deflate-raw",hash12:fields[7],fileName:sanitizeFileName(fields[8])};
    }
    if(type==="A"){
      if(fields.length!==5||!PROFILE_BY_CODE[fields[4]])throw new Error("Invalid ACCEPT.");
      return {type:"accept",id,windowSize:fromBase36(fields[3]),profile:PROFILE_BY_CODE[fields[4]]};
    }
    if(type==="D"){
      if(fields.length!==6||!/^[0-9A-F]{4}$/.test(fields[4])||!/^[A-Z2-7]+$/.test(fields[5]))throw new Error("Invalid DATA block.");
      return {type:"data",id,sequence:fromBase36(fields[3]),crc:fields[4],payload:fields[5]};
    }
    if(type==="E"||type==="K"){
      if(fields.length!==4)throw new Error(`Invalid ${type==="E"?"END":"ACK"}.`);
      return {type:type==="E"?"end":"ack",id,sequence:fromBase36(fields[3])};
    }
    if(type==="Q"){
      if(fields.length!==3)throw new Error("Invalid QUERY.");
      return {type:"query",id};
    }
    if(type==="N"){
      const multipart=/^(\d+)\/(\d+)$/.exec(fields[3]);
      if(fields.length!==(multipart?5:4))throw new Error("Invalid NACK.");
      const part=multipart?Number(multipart[1]):1,parts=multipart?Number(multipart[2]):1;
      if(part<1||parts<1||part>parts||parts>32)throw new Error("Invalid multipart NACK.");
      return {type:"nack",id,part,parts,sequences:parseRanges(fields[multipart?4:3])};
    }
    if(type==="C"){
      if(fields.length!==4||!/^[A-Z2-7]{12}$/.test(fields[3]))throw new Error("Invalid COMPLETE.");
      return {type:"complete",id,hash12:fields[3]};
    }
    if(type==="X"||type==="R"){
      if(fields.length!==4||!/^[A-Z]+$/.test(fields[3]))throw new Error(`Invalid ${type==="X"?"CANCEL":"REJECT"}.`);
      return {type:type==="X"?"cancel":"reject",id,reason:fields[3]};
    }
    throw new Error("Unsupported ~F1 message type.");
  }
  function decodeDataMessage(message,expectedLength){
    if(!message||message.type!=="data"||!/^[0-9A-F]{4}$/.test(message.crc))throw new Error("Invalid DATA block.");
    const bytes=base32Decode(message.payload);
    if(expectedLength!=null&&bytes.length!==expectedLength)throw new Error("DATA block length mismatch.");
    if(crcHex(bytes)!==message.crc)throw new Error("DATA block CRC mismatch.");
    return bytes;
  }
  async function verifyReceived(record){
    const manifestBlock=record.blocks&&record.blocks[0];
    if(!manifestBlock)throw new Error("File manifest block is missing.");
    const chunks=[];for(let sequence=1;sequence<=record.blockCount;sequence+=1){if(!record.blocks[sequence])throw new Error(`Block ${sequence} is missing.`);chunks.push(bytesOf(record.blocks[sequence]));}
    const joined=new Uint8Array(chunks.reduce((total,item)=>total+item.length,0));let offset=0;for(const chunk of chunks){joined.set(chunk,offset);offset+=chunk.length;}
    if(joined.length!==record.originalSize)throw new Error("Reassembled file size mismatch.");
    const digest=await sha256(joined);
    if(hex(digest)!==hex(manifestBlock))throw new Error("Reassembled file SHA-256 mismatch.");
    const hash12=base32Encode(digest).slice(0,12);
    if(hash12!==record.hash12)throw new Error("OFFER hash does not match the manifest.");
    return {bytes:joined,sha256Hex:hex(digest),hash12};
  }
  function sessionForStorage(record){
    const clone={...record,updatedAt:Date.now()};
    if(clone.blocks)clone.blocks=clone.blocks.map(item=>item==null?null:bytesOf(item));
    if(clone.fileBytes)clone.fileBytes=bytesOf(clone.fileBytes);
    return clone;
  }
  class TransferStore{
    constructor(indexedDBValue=globalThis.indexedDB){this.indexedDB=indexedDBValue;this.dbPromise=null;}
    open(){
      if(!this.indexedDB)return Promise.reject(new Error("IndexedDB is not available."));
      if(this.dbPromise)return this.dbPromise;
      this.dbPromise=new Promise((resolve,reject)=>{const request=this.indexedDB.open(DB_NAME,DB_VERSION);request.onupgradeneeded=()=>{const db=request.result;if(!db.objectStoreNames.contains(STORE_NAME))db.createObjectStore(STORE_NAME,{keyPath:"id"});};request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error||new Error("Unable to open transfer storage."));});
      return this.dbPromise;
    }
    async transaction(mode,operation){const db=await this.open();return new Promise((resolve,reject)=>{const tx=db.transaction(STORE_NAME,mode),store=tx.objectStore(STORE_NAME),request=operation(store);request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);});}
    save(record){return this.transaction("readwrite",store=>store.put(sessionForStorage(record)));}
    get(id){return this.transaction("readonly",store=>store.get(id));}
    all(){return this.transaction("readonly",store=>store.getAll());}
    delete(id){return this.transaction("readwrite",store=>store.delete(id));}
  }
  return {BASE32,PROTOCOL_PREFIX,DEFAULTS,PROFILES,PROFILE_BY_CODE,PROFILE_BY_SUBMODE,
    TransferStore,toBase36,fromBase36,base32Encode,base32Decode,crc16Ccitt,crcHex,
    sha256,sha256Fallback,hex,createTransferId,sanitizeFileName,normalizeCallsign,profileForSubmode,
    estimateDuration,enforceFileLimit,prepareBytes,encodeOffer,encodeAccept,encodeData,
    encodeEnd,encodeAck,encodeQuery,encodeComplete,encodeCancel,encodeReject,
    compactRanges,parseRanges,encodeNacks,parseMessage,decodeDataMessage,verifyReceived,
    sessionForStorage};
});
