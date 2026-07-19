// JS8Call email gateway profiles, validation and payload formatting.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Email = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const STORAGE_KEY = "ic705.data.js8-email-gateways.v1";
  const TARGET_RE = /^(@[A-Z0-9/]+|[A-Z0-9/]+(?:>[A-Z0-9/]+)*)$/;
  const EMAIL_RE = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  const FORMATS = ["direct", "direct-msg", "template", "aprs-email2"];
  const POLICIES = ["js8", "ascii", "aprs"];
  const REQUIRED_TEMPLATE_FIELDS = ["{TARGET}", "{EMAIL}", "{BODY}"];
  const APRS_MESSAGE_TEXT_LIMIT = 67;

  function createId() {
    if (typeof crypto !== "undefined" && crypto.randomUUID)
      return `gateway-${crypto.randomUUID()}`;
    return `gateway-${Date.now().toString(36)}-${Math.random().toString(36).slice(2,8)}`;
  }

  function normalizeTarget(value) {
    const target=String(value||"").trim().toUpperCase();
    if(!TARGET_RE.test(target))
      throw new Error("Invalid gateway callsign, group or relay path.");
    return target;
  }

  function validateEmail(value) {
    const email=String(value||"").trim();
    if(!email)return "Enter an email address.";
    if(email.length>254)return "The email address is too long.";
    if(!EMAIL_RE.test(email))return "Enter a valid email address.";
    return null;
  }

  function normalizeGateway(input) {
    const source=input&&typeof input==="object"?input:{};
    const name=String(source.name||"").replace(/\s+/g," ").trim();
    if(!name)throw new Error("Enter a gateway name.");
    const target=normalizeTarget(source.target);
    const dialFrequencyHz=Number(source.dialFrequencyHz);
    if(!Number.isInteger(dialFrequencyHz)||dialFrequencyHz<=0)
      throw new Error("Dial frequency must be a positive whole number in Hz.");
    const offsetHz=Number(source.offsetHz);
    if(!Number.isInteger(offsetHz)||offsetHz<500||offsetHz>2700)
      throw new Error("Audio offset must be between 500 and 2700 Hz.");
    const format=FORMATS.includes(source.format)?source.format:"direct";
    const template=String(source.template||"").trim();
    if(format==="template"){
      if(!template)throw new Error("Enter a gateway template.");
      if(/[\x00-\x1f\x7f]/.test(template))throw new Error("Template must be a single line without control characters.");
      for(const field of REQUIRED_TEMPLATE_FIELDS)
        if(!template.includes(field))throw new Error(`Template must contain ${field}.`);
    }
    const maxBodyLength=Number(source.maxBodyLength);
    if(!Number.isInteger(maxBodyLength)||maxBodyLength<1||maxBodyLength>500)
      throw new Error("Maximum message length must be between 1 and 500.");
    const characterPolicy=POLICIES.includes(source.characterPolicy)
      ?source.characterPolicy:"js8";
    return {id:String(source.id||createId()),name,target,dialFrequencyHz,offsetHz,
      format,template:format==="template"?template:undefined,maxBodyLength,
      characterPolicy,enabled:source.enabled!==false,createdByUser:true};
  }

  function load(storage) {
    try {
      const parsed=JSON.parse(storage?.getItem(STORAGE_KEY)||"[]");
      if(!Array.isArray(parsed))return [];
      const profiles=[];
      for(const item of parsed){try{profiles.push(normalizeGateway(item));}catch(_error){}}
      return profiles;
    } catch(_error) { return []; }
  }

  function save(storage, gateways) {
    const profiles=gateways.map(normalizeGateway);
    storage?.setItem(STORAGE_KEY,JSON.stringify(profiles));
    return profiles;
  }

  function normalizeBody(value, maxLength, policy="js8") {
    let body=String(value||"").replace(/[\r\n\t]+/g," ").replace(/\s+/g," ").trim();
    body=body.normalize("NFKD").replace(/[\u0300-\u036f]/g,"").toUpperCase();
    const allowed=policy==="aprs"?/[^A-Z0-9 .,?!/+\-]/g:/[^A-Z0-9 .,?!/+'"()=:_&$%#@*><[\]{}|;^`~\-]/g;
    body=body.replace(allowed,"").replace(/\s+/g," ").trim();
    if(!body)throw new Error("Enter a message.");
    if(body.length>maxLength)throw new Error(`Message can contain at most ${maxLength} characters.`);
    return body;
  }

  function getBodyBudget(gateway, email) {
    if(!gateway)return 0;
    if(gateway.format==="aprs-email2")
      return Math.max(0,Math.min(gateway.maxBodyLength,
        APRS_MESSAGE_TEXT_LIMIT-String(email||"").trim().length-1));
    return gateway.maxBodyLength;
  }

  function applyTemplate(template,gateway,email,body) {
    return template.replaceAll("{TARGET}",gateway.target)
      .replaceAll("{EMAIL}",email).replaceAll("{BODY}",body)
      .replaceAll("{DIAL_HZ}",String(gateway.dialFrequencyHz))
      .replaceAll("{OFFSET_HZ}",String(gateway.offsetHz));
  }

  function formatGatewayMessage(gateway,request) {
    const email=String(request.recipientEmail||"").trim().toUpperCase();
    const body=request.body;
    let payload="";
    if(gateway.format==="direct")payload=`${gateway.target} EMAIL ${email} ${body}`;
    else if(gateway.format==="direct-msg")payload=`${gateway.target} MSG EMAIL ${email} ${body}`;
    else if(gateway.format==="template")payload=applyTemplate(gateway.template,gateway,email,body);
    else if(gateway.format==="aprs-email2")payload=`${gateway.target} CMD :${"EMAIL-2".padEnd(9," ")}:${email} ${body}`;
    else throw new Error("Unsupported gateway format.");
    if(payload!==gateway.target&&!payload.startsWith(`${gateway.target} `)&&
        !payload.startsWith(`${gateway.target}>`))
      throw new Error("Formatted payload must start with the gateway target.");
    return payload;
  }

  function buildDraft(gateway,emailValue,bodyValue) {
    if(!gateway||gateway.enabled===false)throw new Error("Select an enabled gateway.");
    const recipientEmail=String(emailValue||"").trim();
    const emailError=validateEmail(recipientEmail);
    if(emailError)throw new Error(emailError);
    const budget=getBodyBudget(gateway,recipientEmail);
    if(budget<1)throw new Error("The email address leaves no room for an APRS message.");
    const body=normalizeBody(bodyValue,budget,gateway.characterPolicy);
    const payload=formatGatewayMessage(gateway,{recipientEmail,body});
    return {gateway,recipientEmail:recipientEmail.toUpperCase(),body,budget,payload};
  }

  function transportParts(payload,target) {
    const firstHop=target.split(">")[0];
    if(!payload.startsWith(firstHop))throw new Error("Payload does not start with its gateway target.");
    return {toCall:firstHop,text:payload.slice(firstHop.length).trimStart()};
  }

  return {STORAGE_KEY,FORMATS,POLICIES,normalizeTarget,validateEmail,
    normalizeGateway,normalizeBody,getBodyBudget,formatGatewayMessage,
    buildDraft,transportParts,load,save};
});
