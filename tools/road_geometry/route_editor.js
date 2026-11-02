/* Scene editor owns route intent, history, previews and saved recipe operations.
 * The scene template supplies data, DOM access, map selection and rendering.
 * RoutePlanner owns pure reference construction; Python verifies saved intent.
 */
/* Direct save/load through the optional local export-folder server. */
let localFileApi=false;
function sameHashes(a,b){return a&&b&&Object.keys(a).length===Object.keys(b).length&&Object.keys(b).every(k=>a[k]===b[k])}
async function refreshSelections(){
 if(!localFileApi)return;
 const response=await fetch('/api/selections');if(!response.ok)throw Error('Cannot list the export selections');
 const result=await response.json(),list=$('selectionList');list.replaceChildren();
 const blank=document.createElement('option');blank.value='';blank.textContent='Choose a saved selection…';list.append(blank);
 for(const f of result.files){const o=document.createElement('option');o.value=f.kind+'/'+f.name;o.textContent=(f.kind==='tracks'?'Track · ':'Area · ')+f.name;list.append(o)}
}
async function saveSelection(kind,recipeDocument){
 const name=$('selectionName').value.trim().replace(/\.json$/,'');
 if(!/^[A-Za-z0-9][A-Za-z0-9_.-]{0,79}$/.test(name))throw Error('Enter a name using letters, numbers, underscores or hyphens.');
 if(localFileApi){
  const response=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind,name,document:recipeDocument})});
  const result=await response.json();if(!response.ok)throw Error(result.error||'Save failed');
  $('fileStatus').textContent='Saved '+result.path+' · previous versions are kept automatically.';
  await refreshSelections();$('selectionList').value=kind+'/'+result.name;
 }else{
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(recipeDocument,null,2)],{type:'application/json'}));a.download=name+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);
  $('fileStatus').textContent='Downloaded '+name+'.json. To save directly into this export, open it with the local serve command.';
 }
}
$('refreshSelections').onclick=()=>refreshSelections().catch(e=>$('fileStatus').textContent=e.message);
$('loadSelection').onclick=async()=>{
 const nameDisabled=$('selectionName').disabled,saveDisabled=$('pathSave').disabled;
 $('loadSelection').disabled=true;$('selectionName').disabled=true;$('pathSave').disabled=true;
 try{
  const value=$('selectionList').value;if(!value)throw Error('Choose a saved selection first.');
  const [kind,name]=value.split('/'),response=await fetch('/api/selection?kind='+encodeURIComponent(kind)+'&name='+encodeURIComponent(name));
  const document=await response.json();if(!response.ok)throw Error(document.error);
  if(kind==='tracks')loadTrackRecipe(document);
  else{
   if(!sameHashes(document.source_files_sha256,meta.files_sha256))throw Error('Area belongs to another export.');
   const b=document.bounds_enu_m;
   for(const f of fields)f.input.value=b[(f.side==='min'?0:3)+f.i];roi=true;
   planLow=b.slice(0,3);planHigh=b.slice(3);draw();
  }
  $('selectionName').value=name;$('fileStatus').textContent='Opened selections/'+kind+'/'+name+'.json';
 }catch(e){$('fileStatus').textContent=e.message;}
 finally{$('loadSelection').disabled=false;$('selectionName').disabled=nameDisabled;$('pathSave').disabled=saveDisabled;}
};
(async()=>{
 try{
  if(!['http:','https:'].includes(location.protocol))throw Error('offline');
  const response=await fetch('/api/info');if(!response.ok)throw Error('no file API');
  const info=await response.json();localFileApi=true;
  $('fileStatus').textContent='Local save folder: '+info.save_directory;
  await refreshSelections();
 }catch(e){$('fileStatus').textContent='Download mode. Run “cli.py serve EXPORT_FOLDER” to save/reopen files directly in the WSL/Linux export folder.';}
})();

/* Interactive route intent. Saved reference paths are not measured surfaces. */
const pathState={waypoints:[],selected:-1,history:[],future:[],preview:null,error:''};
let previousEditorSnapshot=null;
let snapPreview=null,snapFrame=null,pendingSnapXY=null;
let pathLineBuffer=null,pathLeftBuffer=null,pathRightBuffer=null,pathWayBuffer=null;
const graph=data.navigation&&Array.isArray(data.navigation.nodes)?data.navigation:null;
function currentGraph(){
 if(!graph)return null;
 if(!$('pathGuide').checked||!data.navigation_geometry)return graph;
 return{...graph,edges:graph.edges.map((e,i)=>({...e,geometry:data.navigation_geometry.edges[i]}))};
}
$('pathZ').value=detailPoints.length?detailPoints[2].toFixed(3):((low[2]+high[2])/2).toFixed(3);
if(!graph||!graph.edges.length){$('pathMode').value='free';$('pathMode').options[0].disabled=true;}
const editorFields=['pathMode','pathGuide','pathJoins','pathClosed','pathSmooth','pathLeft','pathRight','pathMargin','pathSpacing','pathLateral','pathZ','pathLayer','pathAbove','pathBelow'];
function snapshotPath(){return{waypoints:JSON.parse(JSON.stringify(pathState.waypoints)),settings:Object.fromEntries(editorFields.map(id=>[id,$(id).type==='checkbox'?$(id).checked:$(id).value]))}}
function restorePath(s){pathState.waypoints=s.waypoints;for(const [id,v] of Object.entries(s.settings))if($(id).type==='checkbox')$(id).checked=v;else $(id).value=v;pathState.selected=-1;rebuildPath()}
function rememberPath(){pathState.history.push(snapshotPath());pathState.future=[];}
function pathSettings(){
 const s=Object.fromEntries(['Left','Right','Margin','Spacing','Lateral','Z','Layer','Above','Below'].map(k=>[k.toLowerCase(),Number($('path'+k).value)]));
 if(!Object.values(s).every(Number.isFinite)||s.left<=0||s.right<=0||s.margin<0||s.spacing<=0||s.lateral<=0||s.layer<=0||s.above<=0||s.below<=0)throw Error('Extents, spacings and ray bands must be positive; margin can be zero.');
 return s;
}
function pathAnchor(xy,z){
 if($('pathMode').value==='free')return{xyz:[xy[0],xy[1],z]};
 if(!graph)throw Error('Run prepare to acquire the native navigation graph first.');
 const maxXY=15*(devicePixelRatio||1)/planTransform.s;
 return RoutePlanner.snap(currentGraph(),[xy[0],xy[1],z],maxXY,Number($('pathLayer').value));
}
function handlePathClick(end,start){
 try{
  const z=Number($('pathZ').value);if(!Number.isFinite(z))throw Error('Set a finite reference Z.');
  if($('clickAction').value==='move'){
   let best=15*(devicePixelRatio||1)/planTransform.s,index=-1;
   pathState.waypoints.forEach((p,i)=>{const d=Math.hypot(p.xyz[0]-start[0],p.xyz[1]-start[1]);if(d<best){best=d;index=i}});
   if(index<0)throw Error('Start the drag on a waypoint to move it.');
   const replacement=pathAnchor(end,pathState.waypoints[index].xyz[2]);rememberPath();pathState.waypoints[index]=replacement;pathState.selected=index;
  }else{
   const p=pathAnchor(end,z);rememberPath();pathState.waypoints.push(p);pathState.selected=pathState.waypoints.length-1;
   $('pickFeedback').textContent=$('pathMode').value==='free'?'Authored point placed · no road snapping.':`Point snapped ${p.distance_xy.toFixed(2)} m to ${p.source_role==='evaluated_guide'?'evaluated road guide':'navigation link (approximate)'}.`;
  }
  rebuildPath();
 }catch(e){$('pathStatus').textContent=e.message;}
}
function updateSnapPreview(xy){
 pendingSnapXY=xy;if(snapFrame!==null)return;
 snapFrame=requestAnimationFrame(()=>{snapFrame=null;snapPreview=null;
  if($('clickAction').value!=='add'||$('pathMode').value!=='network')return;
  try{const p=pathAnchor(pendingSnapXY,Number($('pathZ').value));snapPreview={...p,click:pendingSnapXY};$('pickFeedback').textContent=`${p.source_role==='evaluated_guide'?'Evaluated road guide':'Approximate navigation link'} · snap ${p.distance_xy.toFixed(2)} m\nClick to add; zoom or adjust level height if this is the wrong source.`;}
  catch(e){$('pickFeedback').textContent=e.message;}
  drawPlan();
 });
}
function corridorXY(points,left,right,closed=false){
 const a=[],b=[];for(let i=0;i<points.length;i++){
  const p=closed&&(i===0||i===points.length-1)?points.at(-2):points[Math.max(0,i-1)],q=closed&&(i===0||i===points.length-1)?points[1]:points[Math.min(points.length-1,i+1)],d=sub(q,p),length=Math.hypot(d[0],d[1]);
  if(length<1e-9)throw Error('Reference path has an undefined horizontal tangent.');
  const n=[-d[1]/length,d[0]/length,0];a.push(add(points[i],scale(n,left)));b.push(add(points[i],scale(n,-right)));
 }return{left:a,right:b};
}
function rebuildPath(){
 pathState.preview=null;pathState.error='';
 if($('pathMode').value==='network')$('pathSmooth').checked=false;
 for(const b of [pathLineBuffer,pathLeftBuffer,pathRightBuffer,pathWayBuffer])if(b)gl.deleteBuffer(b);
 pathLineBuffer=pathLeftBuffer=pathRightBuffer=pathWayBuffer=null;
 try{
  const s=pathSettings();
  if(pathState.waypoints.length>=2){
   const closed=$('pathClosed').checked;
   const p=data.extracted_track?{points:data.initial_recipe.reference_xyz,sampled:data.initial_recipe.reference_xyz,closed:data.initial_recipe.topology==='closed',total_length_m:data.initial_recipe.reference_distance_xyz_m.at(-1)}:null;
   if(p)pathState.preview=p;
   else{let path=$('pathMode').value==='network'?RoutePlanner.network(currentGraph(),pathState.waypoints,closed):RoutePlanner.free(pathState.waypoints.map(p=>p.xyz),closed,$('pathSmooth').checked);if(path.kind==='native_evaluated_guides'&&$('pathJoins').checked)path=RoutePlanner.shortNodeTransitions(path);pathState.preview=RoutePlanner.sample(path,s.spacing);}
   const limits=corridorXY(pathState.preview.sampled,s.left+s.margin,s.right+s.margin,pathState.preview.closed);
   pathLineBuffer=buffer(pathState.preview.sampled.flatMap(p=>[p[0],p[1],p[2]+.15]));
   pathLeftBuffer=buffer(limits.left.flatMap(p=>[p[0],p[1],p[2]+.15]));
   pathRightBuffer=buffer(limits.right.flatMap(p=>[p[0],p[1],p[2]+.15]));
  }
 }catch(e){pathState.error=e.message;}
 pathWayBuffer=buffer(pathState.waypoints.flatMap(p=>[p.xyz[0],p.xyz[1],p.xyz[2]+.25]));
 $('waypoints').replaceChildren();
 pathState.waypoints.forEach((p,i)=>{
  const row=document.createElement('div'),select=document.createElement('button');select.textContent=(pathState.selected===i?'▶ ':'')+(i+1)+': '+p.xyz.map(v=>v.toFixed(2)).join(', ');select.onclick=()=>{pathState.selected=i;rebuildPath();draw()};row.append(select);
  const label=document.createElement('label');label.textContent='Reference Z ';const z=document.createElement('input');z.type='number';z.step='.1';z.value=p.xyz[2].toFixed(3);z.disabled=$('pathMode').value==='network';
  z.onchange=()=>{if(!Number.isFinite(Number(z.value)))return;rememberPath();p.xyz[2]=Number(z.value);rebuildPath();draw()};label.append(z);row.append(label);$('waypoints').append(row);
 });
 const p=pathState.preview;
 const evaluated=$('pathMode').value==='network'&&$('pathGuide').checked&&!!data.navigation_geometry;
 let quality='';if(evaluated&&p&&p.edgeIds){const fallback=new Set(p.edgeIds.filter((_,i)=>p.segmentRoles[i]==='native_navigation_chord')).size,connectors=p.segmentRoles.filter(r=>r==='native_graph_endpoint_connector').length;quality=` · ${fallback} approximate link${fallback===1?'':'s'}${connectors?' · '+connectors+' junction joins':''}`;}
 if(p?.reference_operations){const edits=p.reference_operations[0].transitions.length;quality+=` · ${edits} recorded reference transition${edits===1?'':'s'}${p.transition_warnings.length?' · '+p.transition_warnings.length+' joins need review':''}`;}
 $('pathStatus').textContent=pathState.error||`${pathState.waypoints.length} waypoints${p?' · '+p.total_length_m.toFixed(1)+' m · '+p.sampled.length+' stations':''}${quality}\n${$('pathMode').value==='network'?(evaluated?'Following evaluated guide curves; native graph supplies connectivity.':'Legacy straight navigation links: reference may cut across road curves.'):'Authored reference intent. Surface geometry is measured during extraction.'}`;
 $('pathStatus').classList.toggle('error',!!pathState.error);
 $('sourceHint').textContent=$('pathMode').value==='free'?'Points stay where you place them. Smoothness is an explicit reference-path choice.':evaluated?'Points snap to evaluated guides. The checked reference option transitions bounded offsets within the same guide; other joins remain flagged.':'Points snap to straight graph links. Enable evaluated curves to follow the road definitions more closely.';
 $('pointCount').textContent='Waypoints ('+pathState.waypoints.length+')';
 $('pathGuide').disabled=$('pathMode').value==='free'||!data.navigation_geometry||!!data.extracted_track;
 $('pathSmooth').disabled=$('pathMode').value==='network'||!!data.extracted_track;
 $('pathJoins').disabled=!evaluated||!!data.extracted_track;
 previousEditorSnapshot=snapshotPath();
}
function drawAuthored3D(){
 const p=pathState.preview;if(p){for(const [b,c] of [[pathLineBuffer,[1,.25,.8]],[pathLeftBuffer,[.3,.8,1]],[pathRightBuffer,[.3,.8,1]]]){bind(position,b);fixedColor(c);gl.drawArrays(gl.LINE_STRIP,0,p.sampled.length)}}
 if(pathWayBuffer){bind(position,pathWayBuffer);fixedColor([1,.4,.8]);gl.drawArrays(gl.POINTS,0,pathState.waypoints.length)}
}
function drawAuthoredPlan(ctx,pt){
 const p=pathState.preview;
 if(p){const s=pathSettings(),limits=corridorXY(p.sampled,s.left+s.margin,s.right+s.margin,p.closed);ctx.strokeStyle='#6cdaff';ctx.fillStyle='#58bcff22';ctx.beginPath();[...limits.left,...limits.right.slice().reverse()].forEach((p,i)=>{const xy=pt(p);i?ctx.lineTo(...xy):ctx.moveTo(...xy)});ctx.closePath();ctx.fill();ctx.stroke();ctx.strokeStyle='#ff63d1';ctx.lineWidth=2.5;ctx.beginPath();p.sampled.forEach((p,i)=>{const xy=pt(p);i?ctx.lineTo(...xy):ctx.moveTo(...xy)});ctx.stroke();ctx.lineWidth=1;}
 pathState.waypoints.forEach((p,i)=>{const xy=pt(p.xyz);ctx.fillStyle=i===pathState.selected?'#fff':'#ff63d1';ctx.beginPath();ctx.arc(...xy,5,0,7);ctx.fill();ctx.font='14px sans-serif';ctx.fillText(String(i+1),xy[0]+6,xy[1]-6)});
 if(data.extracted_track)for(const i of data.extracted_track.report.inspection_station_ids||[]){const xy=pt(data.extracted_track.reference[i]);ctx.fillStyle='#ff644f';ctx.beginPath();ctx.arc(...xy,4,0,7);ctx.fill();}
 if(snapPreview){const p=pt(snapPreview.xyz),q=pt(snapPreview.click);ctx.strokeStyle=snapPreview.source_role==='evaluated_guide'?'#6cdaff':'#ffb666';ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(...q);ctx.lineTo(...p);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.arc(...p,7,0,7);ctx.stroke();
  if(snapPreview.guide_id!==null&&data.guides[snapPreview.guide_id]){ctx.strokeStyle='#6cdaff';ctx.beginPath();data.guides[snapPreview.guide_id].forEach((e,i)=>{const x=pt(e[1]);i?ctx.lineTo(...x):ctx.moveTo(...x)});ctx.stroke();}
 }
}
for(const id of editorFields)$(id).onchange=()=>{
 // Changing mode changes the meaning of anchors; never reuse free points as
 // graph node references or silently snap previously authored points.
 if(id==='pathMode'&&pathState.waypoints.length){$('pathStatus').textContent='Clear the current path before changing its source mode.';$(id).value=pathState.waypoints[0].edge?'network':'free';return;}
 if(previousEditorSnapshot&&JSON.stringify(previousEditorSnapshot)!==JSON.stringify(snapshotPath())){pathState.history.push(previousEditorSnapshot);pathState.future=[];}
 if(id==='pathGuide'&&$('pathMode').value==='network'&&pathState.waypoints.length){
  try{const g=currentGraph();pathState.waypoints=pathState.waypoints.map(p=>{const edge=g.edges.find(e=>e.from===p.edge[0]&&e.to===p.edge[1]);return RoutePlanner.snap({...g,edges:[edge]},p.xyz,Infinity,Number($('pathLayer').value));});}
  catch(e){$('pathGuide').checked=!$('pathGuide').checked;$('pathStatus').textContent=e.message;return;}
  $('pickFeedback').textContent=$('pathGuide').checked?'Existing waypoints projected onto evaluated guides. Review the reference and save a named copy.':'Using approximate straight navigation links.';
 }
 rebuildPath();draw();
};
$('pathUndo').onclick=()=>{if(!pathState.history.length)return;pathState.future.push(snapshotPath());restorePath(pathState.history.pop());draw()};
$('pathRedo').onclick=()=>{if(!pathState.future.length)return;pathState.history.push(snapshotPath());restorePath(pathState.future.pop());draw()};
$('pathRemove').onclick=()=>{if(pathState.selected<0)return;rememberPath();pathState.waypoints.splice(pathState.selected,1);pathState.selected=-1;rebuildPath();draw()};
$('pathReverse').onclick=()=>{rememberPath();pathState.waypoints.reverse();pathState.selected=-1;rebuildPath();draw()};
$('pathClear').onclick=()=>{rememberPath();pathState.waypoints=[];pathState.selected=-1;rebuildPath();draw()};
$('pathSave').onclick=async()=>{
 try{
  if(!pathState.preview)throw Error(pathState.error||'Add a valid route before saving.');
  const s=pathSettings(),p=pathState.preview;
  const recipe={schema_version:p.reference_operations?'0.5-track-recipe':p.kind==='native_evaluated_guides'?'0.4-track-recipe':'0.3-track-recipe',frame:'local_ENU',source_mission:meta.source.mission,source_files_sha256:meta.files_sha256,navigation_sha256:data.navigation_sha256||null,
   navigation_geometry_sha256:p.kind==='native_evaluated_guides'?data.navigation_geometry_sha256:null,
   navigation_geometry_file:p.kind==='native_evaluated_guides'?data.navigation_geometry_file:null,
   focus_xyz:p.sampled[Math.floor(p.sampled.length/2)],mode:$('pathMode').value,topology:p.closed?'closed':'open',
   waypoints:pathState.waypoints,...RoutePlanner.referenceFields(p),settings:{left_extent_m:s.left,right_extent_m:s.right,extra_margin_m:s.margin,station_spacing_m:s.spacing,lateral_spacing_m:s.lateral,ray_above_m:s.above,ray_below_m:s.below},
   editor:snapshotPath(),semantics:'Reference intent and chosen extraction extents. Physical edges and drivability unknown; no automatic closure, gap repair or lap repetition.'};
  await saveSelection('tracks',recipe);
 }catch(e){$('pathStatus').textContent=e.message;}
};
function loadTrackRecipe(recipe){
 if(!['0.3-track-recipe','0.4-track-recipe','0.5-track-recipe'].includes(recipe.schema_version)||!sameHashes(recipe.source_files_sha256,meta.files_sha256)||recipe.source_mission!==meta.source.mission)throw Error('Recipe belongs to a different survey.');
 if(recipe.mode==='network'&&recipe.navigation_sha256!==data.navigation_sha256)throw Error('Navigation snapshot differs.');
 if(recipe.reference_kind==='native_evaluated_guides'&&recipe.navigation_geometry_sha256!==data.navigation_geometry_sha256)throw Error('Evaluated guide snapshot differs. Use cli.py rebuild-reference to explicitly update this recipe; its original extraction still uses the saved snapshot.');
 rememberPath();const editor=JSON.parse(JSON.stringify(recipe.editor));editor.settings.pathGuide=recipe.reference_kind==='native_evaluated_guides';editor.settings.pathJoins=!!recipe.reference_operations?.length;restorePath(editor);draw();
 $('pickFeedback').textContent=recipe.reference_kind==='native_navigation_chords'?'Opened legacy navigation-link recipe. Enable evaluated curves to preview an explicit conversion.':'Selection opened. Continue editing or save a named copy.';
}
$('pathLoad').onchange=async e=>{try{const recipe=JSON.parse(await e.target.files[0].text());loadTrackRecipe(recipe);$('selectionName').value=e.target.files[0].name.replace(/\.json$/,'');}catch(e){$('pathStatus').textContent=e.message;}};
if(data.extracted_track){
 const recipe=data.initial_recipe;
 pathState.waypoints=recipe.waypoints;for(const [id,value] of Object.entries(recipe.editor.settings))if($(id).type==='checkbox')$(id).checked=value;else $(id).value=value;
 for(const id of [...editorFields,'pathUndo','pathRedo','pathRemove','pathReverse','pathClear','pathSave','pathLoad','clickAction'])$(id).disabled=true;
 $('clickAction').value='inspect';
 $('selectionName').closest('.step').classList.add('hidden');$('navOnly').closest('.step').classList.add('hidden');
 $('pickFeedback').textContent='Measured corridor · click near the reference to select a station. Edit the recipe from the source workspace.';
 $('title').textContent=meta.source.mission+' — measured corridor';
}
rebuildPath();
if(data.extracted_track)$('pathStatus').textContent='Measured corridor from the saved recipe. Reopen the source survey to edit route intent.\n'+JSON.stringify(data.extracted_track.report,null,2);
