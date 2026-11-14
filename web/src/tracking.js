export async function setupTracking({api,STATIC_MODE,isBusy,lock,onStart,onResult,save,status,getSettings}){
  const $=id=>document.getElementById(id);let imported=null,latest=null;
  function controls(){const split=!STATIC_MODE&&['maximal-admm','maximal-drs','maximal-qp-drs'].includes($('track-method').value);if(!split){$('track-kernel').value='numpy';$('track-workers').value='1';}$('track-kernel').disabled=isBusy()||!split;$('track-workers').disabled=isBusy()||!split||$('track-kernel').value!=='numba';$('track-upload').disabled=isBusy()||STATIC_MODE;if($('track-kernel').value==='numpy')$('track-workers').value='1';}
  const catalog=await api('/api/trajectories');
  for(const item of catalog.trajectories)$('track-scenario').add(new Option(item.name,item.id));
  for(const [id,label] of Object.entries(catalog.methods))$('track-method').add(new Option(label,id));
  $('track-method').value='maximal-schur';
  function description(){const item=catalog.trajectories.find(t=>t.id===$('track-scenario').value);$('tracking-source').textContent=item.description+(STATIC_MODE?' · measured recordings':' · Python API computes each knot');}
  $('track-scenario').onchange=()=>{imported=null;description();};
  $('track-method').onchange=controls;$('track-kernel').onchange=controls;
  $('track-run').textContent=STATIC_MODE?'Load trajectory recording ↗':'Run trajectory IK ↗';
  $('track-upload').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;if(file.size>2_000_000)throw new Error('Planner input limit is 2 MB');const value=JSON.parse(await file.text());if(value.version!==1||!Array.isArray(value.samples))throw new Error('Expected contact/EE plan version 1');imported=value;$('tracking-source').textContent=`Imported ${file.name} · ${value.samples.length} input knots · ${value.provenance?.kind??'provenance required'}`;}catch(e){status(e.message,true);}};
  $('track-input').onclick=async()=>{try{const meta=await api('/api/trajectories/'+$('track-scenario').value);save($('track-scenario').value+'-contact-ee-input.json',JSON.stringify(imported??meta.plan,null,2),'application/json');}catch(e){status(e.message,true);}};
  $('track-export').onclick=()=>{if(!latest)return;const fields=['index','time_s','phase','status','wall_time_ms','loop_time_ms','position_error_m','rotation_error_rad','contact_position_error_m','contact_rotation_error_rad','joint_limit_violation','exceeds_sample_interval'];const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';const csv=[fields.join(','),...latest.knots.map(k=>fields.map(f=>quote(k[f])).join(','))].join('\n');save($('track-scenario').value+'-knots.csv',csv,'text/csv');};
  $('track-run').onclick=async()=>{
    if(isBusy())return;lock(true);status(STATIC_MODE?'Loading measured trajectory IK…':'Computing IK at every input time knot…');
    try{
      const identifier=$('track-scenario').value;const metadata=await api('/api/trajectories/'+identifier);
      if(imported){metadata.plan=imported;metadata.targets=imported.samples[0].targets;}
      await onStart(metadata);
      const result=await api('/api/trajectory/solve',{...getSettings(),trajectory:identifier,method:$('track-method').value,splitting_backend:$('track-kernel').value,cpu_workers:Number($('track-workers').value),...(imported?{plan:imported}:{})});
      onResult(result);const t=result.tracking;latest=t;$('track-export').disabled=false;$('tracking-results').replaceChildren();
      for(const knot of t.knots){const tr=document.createElement('tr');for(const value of [knot.time_s.toFixed(2),knot.phase+(knot.contacts.length?' · '+knot.contacts.join(', '):''),knot.status,knot.wall_time_ms.toFixed(1),(knot.position_error_m*1000).toFixed(3),(knot.contact_position_error_m*1000).toFixed(3),knot.exceeds_sample_interval?'yes':'no']){const td=document.createElement('td');td.textContent=value;tr.append(td);}$('tracking-results').append(tr);}
      $('tracking-summary').textContent=`${STATIC_MODE?'Recorded · ':''}${t.successes}/${t.samples} knots converged · RMS position ${(t.rms_position_error_m*1000).toFixed(3)} mm · max contact ${(t.max_contact_position_error_m*1000).toFixed(3)} mm · p95 knot ${t.p95_knot_ms.toFixed(1)} ms · ${t.interval_misses} sample intervals exceeded · ${t.plan.provenance.kind}. Knot timing includes setup and validation.`;
    }catch(e){status(e.message,true);$('tracking-summary').textContent=e.message;}
    finally{lock(false);controls();}
  };
  controls();description();return controls;
}
