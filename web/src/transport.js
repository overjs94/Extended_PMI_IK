// The public Pages build replays measured records. The local/API build computes.
export const STATIC_MODE=import.meta.env.VITE_STATIC_REPLAY==='1';
const base=import.meta.env.BASE_URL;
const cache=new Map();
async function data(path){
  const url=base+'data/'+path+'.json.gz';
  if(!cache.has(url))cache.set(url,(async()=>{
    const response=await fetch(url);
    if(!response.ok)throw new Error(`Recorded dataset unavailable (${response.status})`);
    const bytes=new Uint8Array(await response.arrayBuffer());
    const text=bytes[0]===31&&bytes[1]===139
      ?await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))).text()
      :new TextDecoder().decode(bytes);
    return JSON.parse(text);
  })());
  return cache.get(url);
}
export async function api(path,body){
  if(!STATIC_MODE){
    const response=await fetch(path,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});
    const value=await response.json();
    if(!response.ok)throw new Error(typeof value.detail==='string'?value.detail:JSON.stringify(value.detail));
    return value;
  }
  if(path==='/api/catalog')return data('catalog');
  if(path==='/api/research')return data('research');
  if(path==='/api/health')return {status:'ok',execution:'recorded',cpu_kernels:{numpy:false,numba:false}};
  if(path==='/api/deployment')return {public_url:new URL(base,location.origin).href,source:'currently loaded static page'};
  if(path.startsWith('/api/scenarios/'))return data('scenarios/'+encodeURIComponent(path.split('/').at(-1).split('?')[0]));
  if(path==='/api/solve'){
    if(body.seed!==7||body.dt!==.04||body.iterations!==600||body.contraction!==.25||body.preview!==true||body.splitting_backend!=='numpy'||body.cpu_workers!==1)
      throw new Error('Static replay contains fixed seed-7 recordings. Use the Python API to compute changed parameters.');
    return data('runs/'+encodeURIComponent(body.scenario)+'/'+encodeURIComponent(body.method));
  }
  if(path==='/api/benchmark'){
    if(body.seeds.length!==1||body.seeds[0]!==7)throw new Error('Only recorded seed 7 is available in the public viewer');
    const catalog=await data('catalog');const rows=[];
    for(const method of Object.keys(catalog.methods)){
      const result=await data('runs/'+encodeURIComponent(body.scenario)+'/'+encodeURIComponent(method));
      rows.push({scenario:body.scenario,seed:7,method,method_label:result.method_label,status:result.status,failure:result.failure,...result.metrics});
    }
    return {rows,protocol:'Recorded same-initial-state comparisons; no computation performed in this page.'};
  }
  throw new Error('This operation needs the Python backend. GitHub Pages only replays measured records.');
}
