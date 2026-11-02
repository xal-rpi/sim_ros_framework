/* Offline recipe rebuild using exactly the browser's routing/design functions. */
'use strict';
const R=require('./route_planner.js');
function rebuild(input){
 const {recipe,navigation,associations}=input;
 if(recipe.mode!=='network'||recipe.reference_kind!=='native_evaluated_guides')throw Error('Rebuild requires a network recipe using evaluated guides.');
 const graph={...navigation,edges:navigation.edges.map((e,i)=>({...e,geometry:associations.edges[i]}))};
 // Keep the selected directed links: improved projection must not choose a
 // different branch or level just because it happens to be nearby.
 const waypoints=recipe.waypoints.map(p=>{
  const edge=graph.edges.find(e=>e.from===p.edge[0]&&e.to===p.edge[1]);
  if(!edge)throw Error('Selected directed link no longer exists.');
  return R.snap({...graph,edges:[edge]},p.xyz,.9,1);
 });
 const path=R.sample(R.shortNodeTransitions(R.network(graph,waypoints,recipe.topology==='closed')),recipe.settings.station_spacing_m);
 const editor=JSON.parse(JSON.stringify(recipe.editor));
 editor.waypoints=waypoints;editor.settings.pathGuide=true;editor.settings.pathJoins=true;
 return {...recipe,schema_version:'0.5-track-recipe',waypoints,editor,...R.referenceFields(path),
  focus_xyz:path.sampled[Math.floor(path.sampled.length/2)],
  reference_rebuild:{parent_recipe_sha256:input.parent_recipe_sha256,
   parent_navigation_geometry_sha256:recipe.navigation_geometry_sha256,
   waypoint_reprojections:waypoints.map((p,i)=>({source_xyz:recipe.waypoints[i].xyz,reference_xyz:p.xyz,
    displacement_xyz_m:Math.hypot(...p.xyz.map((v,k)=>v-recipe.waypoints[i].xyz[k]))})),
   semantics:'Same directed links, updated local guide association, recorded bounded reference transitions. Surface measurements must be queried again.'}};
}
if(require.main===module){let body='';process.stdin.setEncoding('utf8');process.stdin.on('data',chunk=>body+=chunk);process.stdin.on('end',()=>{try{process.stdout.write(JSON.stringify(rebuild(JSON.parse(body))))}catch(e){process.stderr.write(e.message+'\n');process.exitCode=1}})}
module.exports=rebuild;
