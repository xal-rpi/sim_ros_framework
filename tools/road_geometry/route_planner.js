/* Waypoint intent and native graph routing. No pavement or surface engine.
 * Usable as a browser global and as a Node module for independent checks. */
(function(root) {
'use strict';
const distance=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
const lerp=(a,b,t)=>a.map((x,i)=>x+(b[i]-x)*t);
function polylineInfo(points){const arc=[0];for(let i=1;i<points.length;i++)arc.push(arc.at(-1)+distance(points[i-1],points[i]));return{points,arc,length:arc.at(-1)}}
function atArc(info,value){let i=0;while(i<info.points.length-2&&info.arc[i+1]<value)i++;const den=info.arc[i+1]-info.arc[i];return lerp(info.points[i],info.points[i+1],den?(value-info.arc[i])/den:0)}
function cutPolyline(points,t0,t1){const info=polylineInfo(points),a=t0*info.length,b=t1*info.length;return[atArc(info,a),...points.filter((p,i)=>info.arc[i]>a+1e-8&&info.arc[i]<b-1e-8),atArc(info,b)]}
function geometryRole(geometry,point){
 if(!geometry)return'native_navigation_chord';let best=Infinity,role='native_navigation_chord';
 for(let i=1;i<geometry.points.length;i++){const a=geometry.points[i-1],b=geometry.points[i],d=b.map((v,k)=>v-a[k]),den=d.reduce((s,v)=>s+v*v,0),t=den?Math.max(0,Math.min(1,d.reduce((s,v,k)=>s+(point[k]-a[k])*v,0)/den)):0,dist=distance(lerp(a,b,t),point);if(dist<best){best=dist;role=geometry.roles[i-1]}}
 return role;
}
function project(point,a,b) {
  const dx=b[0]-a[0],dy=b[1]-a[1],den=dx*dx+dy*dy;
  const t=den?Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/den)):0;
  return {t,xyz:lerp(a,b,t)};
}
function snap(graph,point,maxXY,heightTolerance) {
  let best=null;const candidates=[];
  graph.edges.forEach(e=>{
    const line=e.geometry?e.geometry.points:[graph.nodes[e.from].xyz,graph.nodes[e.to].xyz],info=polylineInfo(line);
    let p=null;
    for(let i=1;i<line.length;i++){
      // Prefer actual evaluated guide pieces when available. Connectors still
      // remain in routed geometry, but are not advertised as centerline picks.
      if(e.geometry&&e.geometry.guide_id!==null&&e.geometry.roles[i-1]!=='evaluated_guide')continue;
      const q=project(point,line[i-1],line[i]),d=Math.hypot(q.xyz[0]-point[0],q.xyz[1]-point[1]);
      if(!p||d<p.d)p={...q,d,t:(info.arc[i-1]+q.t*(info.arc[i]-info.arc[i-1]))/info.length};
    }
    if(!p)return;
    const d=Math.hypot(p.xyz[0]-point[0],p.xyz[1]-point[1]);
    if(d>maxXY||Math.abs(p.xyz[2]-point[2])>heightTolerance)return;
    const cost=d+Math.abs(p.xyz[2]-point[2])*.05;
    candidates.push({...p,distance_xy:d});
    if(!best||cost<best.cost)best={...p,edge:[e.from,e.to],cost,distance_xy:d,guide_id:e.geometry?e.geometry.guide_id:null,source_role:e.geometry?(e.geometry.guide_id===null?'navigation_fallback':'evaluated_guide'):'navigation_chord'};
  });
  if(!best)throw Error('No navigation edge within click distance and height tolerance. Choose a level height or use an authored path.');
  if(candidates.some(p=>Math.hypot(p.xyz[0]-best.xyz[0],p.xyz[1]-best.xyz[1])<.25&&Math.abs(p.xyz[2]-best.xyz[2])>.5&&Math.abs(Math.abs(p.xyz[2]-point[2])-Math.abs(best.xyz[2]-point[2]))<.25))throw Error('Ambiguous overlapping navigation levels. Set a more specific waypoint Z or reduce the height tolerance.');
  return best;
}
function shortest(adjacency,start,end) {
  const costs=Array(adjacency.length).fill(Infinity),previous=Array(adjacency.length).fill(null),done=new Set();
  costs[start]=0;
  // The surveyed graphs are small. A simple deterministic scan also makes
  // equal-cost branch choices repeatable; no hidden route repair is applied.
  while(true) {
    let u=-1;
    for(let i=0;i<costs.length;i++)if(!done.has(i)&&(u<0||costs[i]<costs[u]))u=i;
    if(u<0||!Number.isFinite(costs[u]))throw Error('Waypoints are disconnected in the directed navigation graph. No gap was bridged.');
    if(u===end)break;
    done.add(u);
    for(const edge of adjacency[u]) {
      const value=costs[u]+edge.length;
      if(value<costs[edge.to]){costs[edge.to]=value;previous[edge.to]={from:u,edge};}
    }
  }
  const nodes=[end],edges=[];
  while(nodes.at(-1)!==start){const p=previous[nodes.at(-1)];edges.unshift(p.edge.source);nodes.push(p.from);}
  return {nodes:nodes.reverse(),edges};
}
function network(graph,anchors,closed) {
  if(anchors.length<2)throw Error('Add at least two ordered waypoints.');
  const xyz=graph.nodes.map(n=>n.xyz.slice()),groups=new Map(),adjacency=xyz.map(()=>[]);
  anchors.forEach((a,i)=>{
    const [u,v]=a.edge;
    if(!graph.edges.some(e=>e.from===u&&e.to===v))throw Error('Anchor refers to an unavailable directed edge.');
    const key=Math.min(u,v)+':'+Math.max(u,v),t=u<v?a.t:1-a.t;
    if(!groups.has(key))groups.set(key,[]);
    groups.get(key).push({t,id:xyz.length});xyz.push(a.xyz.slice());adjacency.push([]);
  });
  graph.edges.forEach((e,source)=>{
    const a=e.from,b=e.to,key=Math.min(a,b)+':'+Math.max(a,b);
    let cuts=(groups.get(key)||[]).slice().sort((x,y)=>x.t-y.t||x.id-y.id);
    if(a>b)cuts.reverse();
    const chain=[a,...cuts.map(x=>x.id),b];
    const fractions=[0,...cuts.map(c=>a<b?c.t:1-c.t),1];
    for(let j=1;j<chain.length;j++){
      const curve=e.geometry?cutPolyline(e.geometry.points,fractions[j-1],fractions[j]):[xyz[chain[j-1]],xyz[chain[j]]];
      adjacency[chain[j-1]].push({to:chain[j],length:polylineInfo(curve).length,source,curve});
    }
  });
  const first=graph.nodes.length,ordered=[],edgeIds=[],legs=[];
  const legCount=closed?anchors.length:anchors.length-1;
  for(let i=0;i<legCount;i++){
    const path=shortest(adjacency,first+i,first+(i+1)%anchors.length);
    if(i)path.nodes.shift();ordered.push(...path.nodes);edgeIds.push(...path.edges);legs.push(path.edges.length);
  }
  let points=ordered.map(i=>xyz[i]),segmentRoles=edgeIds.map(()=> 'native_navigation_chord');
  if(graph.edges.some(e=>e.geometry)){
    const expanded=[],expandedIds=[],expandedRoles=[];
    let previousStep=null;
    for(let i=0;i<edgeIds.length;i++){
      const edge=adjacency[ordered[i]].find(e=>e.to===ordered[i+1]&&e.source===edgeIds[i]);
      if(!expanded.length)expanded.push(edge.curve[0]);
      let start=1;
      const before=previousStep&&graph.edges[previousStep.source].geometry,after=graph.edges[edge.source].geometry;
      // A merged graph node can sit beside its original guide. If both native
      // links project onto exactly the SAME guide point, keep that continuous
      // guide point instead of an artificial out-and-back spur via the node.
      // No separated endpoints are joined and no source geometry is modified.
      if(before&&after&&before.guide_id!==null&&before.guide_id===after.guide_id&&
         before.roles.at(-1)==='native_graph_endpoint_connector'&&after.roles[0]==='native_graph_endpoint_connector'&&
         previousStep.curve.length>=2&&edge.curve.length>=2&&
         distance(previousStep.curve.at(-2),edge.curve[1])<1e-8&&expanded.length>=2){
        expanded.pop();expandedIds.pop();expandedRoles.pop();start=2;
      }
      for(let j=start;j<edge.curve.length;j++){const p=edge.curve[j];expandedRoles.push(geometryRole(after,lerp(expanded.at(-1),p,.5)));expanded.push(p);expandedIds.push(edge.source)}
      previousStep=edge;
    }
    points=expanded;edgeIds.splice(0,edgeIds.length,...expandedIds);segmentRoles=expandedRoles;
  }
  return {points,edgeIds,segmentRoles,segmentGuideIds:edgeIds.map(i=>graph.edges[i].geometry?.guide_id??null),
    protectedPoints:anchors.map(a=>a.xyz),legs,kind:graph.edges.some(e=>e.geometry)?'native_evaluated_guides':'native_navigation_chords',smoothing:'none',closed};
}

const transitionDefaults={window_m:3,max_connector_length_m:.5,max_heading_deg:30,sample_spacing_m:.1};
function shortNodeTransitions(path,settings=transitionDefaults){
  // This designs a REFERENCE through a graph join. Native guide/map samples
  // remain unchanged. Only two sides of the same known guide are eligible;
  // a disconnected edge, another guide, a layer change or a large offset is
  // never "repaired" by this operation. It neither wraps nor closes a route.
  settings={...transitionDefaults,...settings};
  if(Object.values(settings).some(v=>!Number.isFinite(v)||v<=0)||settings.max_heading_deg>30||settings.max_connector_length_m>.5)
    throw Error('Reference transition policy requires positive settings, at most 0.5 m of connectors and 30 degrees of heading change.');
  const info=polylineInfo(path.points),edits=[],warnings=[],roles=path.segmentRoles||[];
  const tangent=s=>{let i=0;while(i<info.points.length-2&&info.arc[i+1]<=s)i++;const d=info.points[i+1].map((v,k)=>v-info.points[i][k]),l=Math.hypot(...d);return d.map(v=>v/l)};
  for(let first=0;first<roles.length;first++){
    if(roles[first]!=='native_graph_endpoint_connector')continue;
    let last=first;while(roles[last+1]==='native_graph_endpoint_connector')last++;
    const start=info.arc[first]-settings.window_m,end=info.arc[last+1]+settings.window_m;
    const length=info.arc[last+1]-info.arc[first],before=path.segmentGuideIds?.[first-1],after=path.segmentGuideIds?.[last+1];
    let reason=null;
    if(first===0||last===roles.length-1||roles[first-1]!=='evaluated_guide'||roles[last+1]!=='evaluated_guide'||before==null||before!==after)reason='No common evaluated guide on both sides';
    else if(length>settings.max_connector_length_m)reason='Connector group exceeds bounded reference policy';
    else if(start<0||end>info.length||edits.length&&start<edits.at(-1).end_m)reason='Insufficient space for a separate transition';
    else if(Math.ceil((end-start)/settings.sample_spacing_m)>10000)reason='Transition would exceed 10,000 reference samples';
    else if(roles.some((role,i)=>info.arc[i]<end&&info.arc[i+1]>start&&(i<first||i>last)&&
      (role!=='evaluated_guide'||path.segmentGuideIds[i]!==before)))reason='Transition window includes another junction or source';
    else if((path.protectedPoints||[]).some(p=>info.points.some((q,i)=>info.arc[i]>start+1e-8&&info.arc[i]<end-1e-8&&distance(p,q)<1e-8)))reason='Transition would move a selected waypoint';
    else{
      const a=tangent(start),b=tangent(end),cos=a[0]*b[0]+a[1]*b[1],den=Math.hypot(a[0],a[1])*Math.hypot(b[0],b[1]);
      if(!den||Math.acos(Math.max(-1,Math.min(1,cos/den)))*180/Math.PI>settings.max_heading_deg||Math.abs(a[2]-b[2])>.1)reason='Heading or grade change requires explicit route design';
    }
    const segments=Array.from({length:last-first+1},(_,i)=>first+i);
    if(reason)warnings.push({connector_segments:segments,reason});
    else edits.push({start_m:start,end_m:end,connector_segments:segments,sample_count:Math.ceil((end-start)/settings.sample_spacing_m)});
    first=last;
  }
  const result=[];let cursor=0;
  const append=p=>{if(!result.length||distance(result.at(-1),p)>1e-10)result.push(p)};
  for(const edit of edits){
    while(cursor<info.points.length&&info.arc[cursor]<edit.start_m-1e-10)append(info.points[cursor++]);
    const a=atArc(info,edit.start_m),b=atArc(info,edit.end_m),scale=distance(a,b),v0=tangent(edit.start_m).map(x=>x*scale),v1=tangent(edit.end_m).map(x=>x*scale);
    // Quintic Hermite: original endpoint position and chord tangent, zero
    // second derivative at either end. No Surface fitting happens here.
    for(let j=0;j<=edit.sample_count;j++){
      const t=j/edit.sample_count;
      append(a.map((x,k)=>{const d=b[k]-x;return x+v0[k]*t+(10*d-6*v0[k]-4*v1[k])*t**3+(-15*d+8*v0[k]+7*v1[k])*t**4+(6*d-3*v0[k]-3*v1[k])*t**5}));
    }
    while(cursor<info.points.length&&info.arc[cursor]<=edit.end_m+1e-10)cursor++;
  }
  while(cursor<info.points.length)append(info.points[cursor++]);
  return {...path,points:result,sourcePoints:path.points,
    reference_operations:[{op:'short_same_guide_node_transition',version:1,settings,transitions:edits}],transition_warnings:warnings};
}
function free(waypoints,closed,smooth) {
  if(waypoints.length<(closed?3:2))throw Error(closed?'A closed authored path requires at least three waypoints.':'Add at least two ordered waypoints.');
  if(waypoints.some((p,i)=>i&&distance(p,waypoints[i-1])<1e-6))throw Error('Adjacent waypoints coincide.');
  if(!smooth)return {points:closed?[...waypoints,waypoints[0]]:waypoints,kind:'authored_polyline',closed};
  // Centripetal Catmull-Rom is an explicitly chosen REFERENCE-path design.
  // It is not a reconstruction or smoothing of measured road geometry.
  const points=[],n=waypoints.length;
  for(let i=0;i<(closed?n:n-1);i++){
    const p1=waypoints[i],p2=waypoints[(i+1)%n];
    const p0=i?waypoints[i-1]:closed?waypoints[n-1]:p1.map((x,k)=>2*x-p2[k]);
    const p3=i+2<n?waypoints[i+2]:closed?waypoints[(i+2)%n]:p2.map((x,k)=>2*x-p1[k]);
    const t0=0,t1=Math.sqrt(distance(p0,p1)),t2=t1+Math.sqrt(distance(p1,p2)),t3=t2+Math.sqrt(distance(p2,p3));
    const count=Math.max(8,Math.min(4096,Math.ceil(distance(p1,p2)/.2)));
    for(let j=0;j<count;j++){
      const t=t1+(t2-t1)*j/count;
      const a1=lerp(p0,p1,(t-t0)/(t1-t0)),a2=lerp(p1,p2,(t-t1)/(t2-t1)),a3=lerp(p2,p3,(t-t2)/(t3-t2));
      const b1=lerp(a1,a2,(t-t0)/(t2-t0)),b2=lerp(a2,a3,(t-t1)/(t3-t1));
      points.push(lerp(b1,b2,(t-t1)/(t2-t1)));
    }
  }
  points.push(closed?waypoints[0]:waypoints.at(-1));
  return {points,kind:'authored_centripetal_catmull_rom',closed};
}
function sample(path,spacing) {
  if(!Number.isFinite(spacing)||spacing<=0)throw Error('Station spacing must be positive.');
  const points=path.points.filter((p,i)=>!i||distance(p,path.points[i-1])>1e-9);
  if(points.length<2)throw Error('Reference path has zero length.');
  const s=[0];for(let i=1;i<points.length;i++)s.push(s.at(-1)+distance(points[i-1],points[i]));
  const total=s.at(-1),count=Math.ceil(total/spacing);
  if(count>10000)throw Error('Limit a selection to 10,000 stations; increase spacing or shorten it.');
  const result=[],edgeIndex=[],stations=[];let j=0;
  for(let i=0;i<=count;i++){
    const target=Math.min(i*spacing,total);while(j<s.length-2&&s[j+1]<target)j++;
    result.push(lerp(points[j],points[j+1],(target-s[j])/(s[j+1]-s[j])));edgeIndex.push(j);stations.push(target);
  }
  return {...path,sampled:result,sample_edge_index:edgeIndex,distance_xyz_m:stations,total_length_m:total};
}
function referenceFields(path){return{
 reference_kind:path.kind,reference_xyz:path.sampled,reference_distance_xyz_m:path.distance_xyz_m,
 navigation_edge_ids:path.edgeIds||[],source_reference_xyz:path.sourcePoints||path.points,
 reference_polyline_xyz:path.points,source_reference_roles:path.segmentRoles||[],
 reference_operations:path.reference_operations||[],transition_warnings:path.transition_warnings||[]
}}
const api={project,snap,network,free,sample,shortNodeTransitions,transitionDefaults,referenceFields};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.RoutePlanner=api;
})(typeof globalThis!=='undefined'?globalThis:this);
