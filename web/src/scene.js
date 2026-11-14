import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {ColladaLoader} from 'three/addons/loaders/ColladaLoader.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';

export function createScene(container){
  const scene=new THREE.Scene();
  const camera=new THREE.PerspectiveCamera(40,1,.01,100);camera.up.set(0,0,1);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0,0);renderer.outputColorSpace=THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);renderer.domElement.setAttribute('aria-label','Interactive URDF robot visualization');
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.25;controls.maxDistance=12;
  scene.add(new THREE.HemisphereLight(0xdceeff,0x334550,2.7));const key=new THREE.DirectionalLight(0xffffff,3.2);key.position.set(2,-3,5);scene.add(key);const rim=new THREE.DirectionalLight(0x63c6c4,1.7);rim.position.set(-2,3,2);scene.add(rim);
  const grid=new THREE.GridHelper(8,40,0x53606a,0x283743);grid.rotation.x=Math.PI/2;grid.position.z=-.015;scene.add(grid);
  const axes=new THREE.AxesHelper(.22);axes.position.set(-1.2,-1.1,.002);scene.add(axes);
  let robot=new THREE.Group(),decorations=new THREE.Group(),trail=new THREE.Group();scene.add(robot,decorations,trail);
  let frames={},skeleton,model,graspLines=[],loadVersion=0;
  const cache=new Map(),loader=new STLLoader(),collada=new ColladaLoader(),objLoader=new OBJLoader();
  const palette=[0xbacbd0,0x77cdbb,0x8eafc9,0xe5bd87,0xb59dc9,0x98b587,0xbf8e88,0x86bfc7];
  const matrix=T=>new THREE.Matrix4().set(...T.flat());
  function dispose(group){group.traverse(o=>{if(o.geometry&&!o.userData.shared)o.geometry.dispose();if(o.material){for(const m of Array.isArray(o.material)?o.material:[o.material])m.dispose();}});scene.remove(group);}
  let homeTarget=new THREE.Vector3(0,0,.62),homeDistance=4;
  function reset(){camera.position.copy(new THREE.Vector3(2.25,-3,1.5).normalize().multiplyScalar(homeDistance).add(homeTarget));controls.target.copy(homeTarget);controls.update();}
  reset();
  const resize=new ResizeObserver(()=>{const w=container.clientWidth,h=container.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();});resize.observe(container);
  async function load(data,mode='collision'){
    const bounds=new THREE.Box3().setFromPoints(Object.values(data.initial).map(T=>new THREE.Vector3(...T.slice(0,3).map(r=>r[3]))));
    homeTarget=bounds.getCenter(new THREE.Vector3());const size=bounds.getSize(new THREE.Vector3());homeDistance=Math.max(size.x,size.y,size.z,.5)*2.7;
    const version=++loadVersion;model=data;dispose(robot);dispose(decorations);dispose(trail);robot=new THREE.Group();decorations=new THREE.Group();trail=new THREE.Group();scene.add(robot,decorations,trail);frames={};graspLines=[];
    for(const link of data.links){const group=new THREE.Group();group.matrixAutoUpdate=false;robot.add(group);frames[link]=group;}
    const points=[];for(const j of data.joints){if(!j.child.endsWith('_sc'))points.push(new THREE.Vector3(),new THREE.Vector3());}
    skeleton=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0x6dc4b3,transparent:true,opacity:data.name.includes('humanoid')?.9:.2}));robot.add(skeleton);
    const geometryList=mode==='visual'?data.visuals:[...(data.collisions??[]),...data.visuals.filter(v=>!(data.collisions??[]).some(c=>c.link===v.link))];
    const pending=geometryList.map(async v=>{
      let geometry;let shared=false;
      if(v.type==='mesh'){
        if(v.filename.startsWith('/models/external/')){
          const url=v.filename;
          if(url.endsWith('.stl')){
            if(!cache.has(url))cache.set(url,loader.loadAsync(url).then(g=>{g.computeVertexNormals();return g;}));
            geometry=await cache.get(url);shared=true;
          }else{
            if(!cache.has(url))cache.set(url,url.endsWith('.dae')?collada.loadAsync(url).then(c=>{c.scene.quaternion.identity();c.scene.updateMatrix();return c.scene;}):url.endsWith('.obj')?objLoader.loadAsync(url):Promise.resolve(null));
            const template=await cache.get(url);if(!template||version!==loadVersion)return;
            const wrapper=new THREE.Group();wrapper.matrixAutoUpdate=false;wrapper.matrix.copy(matrix(v.origin));
            if(v.scale)wrapper.matrix.scale(new THREE.Vector3(...v.scale.split(/\s+/).map(Number)));
            const visual=template.clone(true);visual.traverse(o=>{if(o.isMesh){o.userData.shared=true;o.material=Array.isArray(o.material)?o.material.map(m=>m.clone()):o.material.clone();if(v.rgba&&!Array.isArray(o.material))o.material.color.setRGB(...v.rgba.slice(0,3));}});wrapper.add(visual);frames[v.link]?.add(wrapper);return;
          }
        }else{
        const match=v.filename.match(/(?:^|\/)link([0-7])\.(?:dae|stl)$/i);
        if(!match||!v.filename.startsWith('package://franka_description/'))return;
        if(mode==='visual'&&v.filename.endsWith('.dae')){
          const url=`/models/franka_description/meshes/visual/link${match[1]}.dae`;
          // URDF defines the mesh coordinate frame. ColladaLoader's automatic
          // file up-axis rotation would rotate every link a second time.
          if(!cache.has(url))cache.set(url,collada.loadAsync(url).then(c=>{c.scene.quaternion.identity();c.scene.updateMatrix();return c.scene;}));
          const template=await cache.get(url);if(version!==loadVersion)return;
          const wrapper=new THREE.Group();wrapper.matrixAutoUpdate=false;wrapper.matrix.copy(matrix(v.origin));
          if(v.scale)wrapper.matrix.scale(new THREE.Vector3(...v.scale.split(/\s+/).map(Number)));
          const visual=template.clone(true);visual.traverse(o=>{if(o.isMesh){o.userData.shared=true;o.material=Array.isArray(o.material)?o.material.map(m=>m.clone()):o.material.clone();}});wrapper.add(visual);frames[v.link]?.add(wrapper);return;
        }
        const url=`/models/franka_description/meshes/collision/link${match[1]}.stl`;
        if(!cache.has(url))cache.set(url,loader.loadAsync(url).then(g=>{g.computeVertexNormals();return g;}));
        try{geometry=await cache.get(url);shared=true;}catch{return;}
        }
      }else if(v.type==='box'){const d=v.size.split(/\s+/).map(Number);geometry=new THREE.BoxGeometry(...d);}
      else if(v.type==='sphere')geometry=new THREE.SphereGeometry(Number(v.radius),24,16);
      else if(v.type==='cylinder'){geometry=new THREE.CylinderGeometry(Number(v.radius),Number(v.radius),Number(v.length),24);geometry.rotateX(Math.PI/2);}
      else return;
      if(version!==loadVersion)return;
      const arm=Number(v.link.match(/^a(\d+)_/)?.[1]??0),object=v.link==='object';
      const color=v.rgba?new THREE.Color(...v.rgba.slice(0,3)):object?0xd6a968:palette[arm%palette.length];
      const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,metalness:.28,roughness:.42,transparent:object,opacity:object?.75:1}));mesh.userData.shared=shared;
      mesh.matrixAutoUpdate=false;mesh.matrix.copy(matrix(v.origin));
      if(v.scale)mesh.matrix.scale(new THREE.Vector3(...v.scale.split(/\s+/).map(Number)));
      frames[v.link]?.add(mesh);
    });
    for(const obstacle of data.obstacles??[]){const mesh=new THREE.Mesh((obstacle.type==='box'?new THREE.BoxGeometry(...obstacle.half_extents.map(x=>x*2)):new THREE.SphereGeometry(obstacle.radius,32,24)),new THREE.MeshStandardMaterial({color:0xd97970,transparent:true,opacity:.55,roughness:.7}));mesh.position.fromArray(obstacle.center);decorations.add(mesh);const wire=new THREE.Mesh(mesh.geometry.clone(),new THREE.MeshBasicMaterial({color:0xec9e8e,wireframe:true,transparent:true,opacity:.1}));wire.position.copy(mesh.position);decorations.add(wire);}
    for(const [name,T] of Object.entries(data.targets??{})){const group=new THREE.Group();group.matrixAutoUpdate=false;group.matrix.copy(matrix(T));group.add(new THREE.AxesHelper(.14));const sphere=new THREE.Mesh(new THREE.SphereGeometry(.017,16,12),new THREE.MeshBasicMaterial({color:0xf1c37e}));group.add(sphere);decorations.add(group);}
    for(const grasp of data.grasps??[]){if(grasp.a==='world')continue;const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),new THREE.LineDashedMaterial({color:0xe0ba7c,dashSize:.025,gapSize:.018,transparent:true,opacity:.5}));decorations.add(line);graspLines.push([line,grasp.a,grasp.b]);}
    update(data.initial);reset();await Promise.all(pending);
  }
  function update(poses){
    if(!model)return;
    for(const [name,T]of Object.entries(poses)){if(frames[name])frames[name].matrix.copy(matrix(T));}
    const a=skeleton.geometry.attributes.position.array;let i=0;
    for(const j of model.joints){if(j.child.endsWith('_sc'))continue;for(const n of [j.parent,j.child]){const p=poses[n]?.slice(0,3).map(r=>r[3])??[0,0,0];a.set(p,i);i+=3;}}
    skeleton.geometry.attributes.position.needsUpdate=true;skeleton.geometry.computeBoundingSphere();
    for(const [line,one,two] of graspLines){if(!poses[one]||!poses[two])continue;const a=line.geometry.attributes.position.array;a.set(poses[one].slice(0,3).map(r=>r[3]),0);a.set(poses[two].slice(0,3).map(r=>r[3]),3);line.geometry.attributes.position.needsUpdate=true;line.geometry.computeBoundingSphere();line.computeLineDistances();}
  }
  function trajectory(result){dispose(trail);trail=new THREE.Group();scene.add(trail);for(const n of Object.keys(result.targets)){const points=result.frames.map(f=>new THREE.Vector3(...f[n].slice(0,3).map(r=>r[3])));const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0x8dcebc,transparent:true,opacity:.55}));trail.add(line);}}
  let paused=false,lastFrame=0;
  renderer.setAnimationLoop(time=>{if(paused||time-lastFrame<40)return;lastFrame=time;controls.update();renderer.render(scene,camera);});
  return{load,update,reset,trajectory,pause:value=>{paused=value;}};
}
