'use strict';
const assert=require('node:assert/strict');
const R=require('./route_planner.js');
const graph={nodes:[{xyz:[0,0,0]},{xyz:[10,0,0]},{xyz:[10,10,0]},{xyz:[0,10,0]}],edges:[{from:0,to:1},{from:1,to:0},{from:1,to:2},{from:2,to:1},{from:2,to:3},{from:3,to:2},{from:3,to:0},{from:0,to:3}]};
const a=R.snap(graph,[2,0,0],.1,.1),b=R.snap(graph,[10,7,0],.1,.1);
const path=R.sample(R.network(graph,[a,b],false),1);
assert.deepEqual(path.points,[[2,0,0],[10,0,0],[10,7,0]]);
assert.deepEqual(path.edgeIds,[0,2]);
assert.deepEqual(path.sampled[0],[2,0,0]);assert.deepEqual(path.sampled.at(-1),[10,7,0]);
// Virtual anchors must split a shared directed edge without snapping to nodes.
assert.deepEqual(R.network(graph,[a,R.snap(graph,[8,0,0],.1,.1)],false).points,[[2,0,0],[8,0,0]]);
const oneWay={nodes:graph.nodes.slice(0,2),edges:[{from:0,to:1}]};
assert.throws(()=>R.network(oneWay,[R.snap(oneWay,[8,0,0],1,1),R.snap(oneWay,[2,0,0],1,1)],false),/disconnected/);
assert.throws(()=>R.snap(graph,[5,0,10],1,1),/height tolerance/);
const stacked={nodes:[{xyz:[0,0,0]},{xyz:[10,0,0]},{xyz:[0,0,4]},{xyz:[10,0,4]}],edges:[{from:0,to:1},{from:2,to:3}]};
assert.throws(()=>R.snap(stacked,[5,0,2],1,3),/Ambiguous overlapping/);
const disconnected={nodes:graph.nodes,edges:[{from:0,to:1},{from:2,to:3}]};
assert.throws(()=>R.network(disconnected,[R.snap(disconnected,[2,0,0],1,1),R.snap(disconnected,[5,10,0],1,1)],false),/disconnected/);
const loop=R.sample(R.free([[0,0,0],[5,0,0],[5,5,0]],true,true),.25);
assert.deepEqual(loop.sampled[0],loop.sampled.at(-1));assert(loop.sampled.every(p=>p.every(Number.isFinite)));
assert.throws(()=>R.free([[0,0,0],[0,0,0]],false,true),/coincide/);
assert.throws(()=>R.sample(path,1e-9),/10,000/);
// Evaluated center-guide curvature must survive routing; do not replace it by
// its graph chord. Picking projects onto the curve itself.
const curved={nodes:[{xyz:[0,0,0]},{xyz:[10,0,0]}],edges:[{from:0,to:1,geometry:{points:[[0,0,0],[5,3,0],[10,0,0]],roles:['evaluated_guide','evaluated_guide'],guide_id:7}}]};
const ca=R.snap(curved,[2.5,1.5,0],.1,.1),cb=R.snap(curved,[7.5,1.5,0],.1,.1);
assert.equal(ca.source_role,'evaluated_guide');
const curvedPoints=R.network(curved,[ca,cb],false).points;
assert.equal(curvedPoints.length,3);
assert(curvedPoints.every((p,i)=>Math.hypot(...p.map((v,k)=>v-[[2.5,1.5,0],[5,3,0],[7.5,1.5,0]][i][k]))<1e-12));
console.log('Route planner: directed routing, virtual anchors, levels, curved guides, disconnections and explicit loop checks passed.');
// Same-guide joins must not create a spur through an offset merged graph node.
const spurs={nodes:[{xyz:[0,0,0]},{xyz:[10,0,0]},{xyz:[20,0,0]}],edges:[
 {from:0,to:1,geometry:{points:[[0,0,0],[2,1,0],[10,1,0],[10,0,0]],roles:['native_graph_endpoint_connector','evaluated_guide','native_graph_endpoint_connector'],guide_id:4}},
 {from:1,to:2,geometry:{points:[[10,0,0],[10,1,0],[18,1,0],[20,0,0]],roles:['native_graph_endpoint_connector','evaluated_guide','native_graph_endpoint_connector'],guide_id:4}}
]};
const sa=R.snap({...spurs,edges:[spurs.edges[0]]},[3,1,0],.1,.1),sb=R.snap({...spurs,edges:[spurs.edges[1]]},[17,1,0],.1,.1);
const joined=R.network(spurs,[sa,sb],false);assert(!joined.points.some(p=>p[0]===10&&p[1]===0));assert(joined.points.some(p=>p[0]===10&&p[1]===1));
assert(joined.segmentRoles.every(r=>r==='evaluated_guide'));
// Small graph joins on the same guide are explicit reference designs, retaining
// the native polyline and source roles for replay. They are not physical gaps.
const offset={points:[[0,0,0],[4,0,0],[4,-.15,0],[4,.05,0],[12,.05,0]],
 segmentRoles:['evaluated_guide','native_graph_endpoint_connector','native_graph_endpoint_connector','evaluated_guide'],
 segmentGuideIds:[7,7,7,7],protectedPoints:[[0,0,0],[12,.05,0]],kind:'native_evaluated_guides',closed:false};
const fixed=R.shortNodeTransitions(offset);
assert.equal(fixed.reference_operations[0].transitions.length,1);
assert.deepEqual(fixed.sourcePoints,offset.points);
assert.deepEqual(fixed.points[0],offset.points[0]);assert.deepEqual(fixed.points.at(-1),offset.points.at(-1));
assert(!fixed.points.some(p=>p[1]<-1e-8));
assert.equal(fixed.transition_warnings.length,0);
const separate=R.shortNodeTransitions({...offset,segmentGuideIds:[7,7,8,8]});
assert.equal(separate.reference_operations[0].transitions.length,0);assert.equal(separate.transition_warnings.length,1);
const waypoint=R.shortNodeTransitions({...offset,protectedPoints:[offset.points[1]]});
assert.equal(waypoint.reference_operations[0].transitions.length,0);
const large=R.shortNodeTransitions({...offset,points:[[0,0,0],[4,0,0],[4,-2,0],[4,.05,0],[12,.05,0]]});
assert.equal(large.reference_operations[0].transitions.length,0);
console.log('Reference transitions: same-guide bounds, retained source, waypoint protection and large-gap refusal passed.');

{
const assert=require('node:assert/strict');
const {rows}=require('./route_selection.js');
const closed={definition:{topology:'closed'},center:[[0,0,0],[2,0,0],[2,2,0],[0,0,0]]};
assert.deepEqual(rows(closed,1,null,'reverse',true),[1,0,2,1]);
assert.deepEqual(rows(closed,2,1,'forward',false),[2,0,1]);
const open={definition:{topology:'open'},center:closed.center.slice(0,3)};
assert.deepEqual(rows(open,2,0,'reverse',false),[2,1,0]);
assert.throws(()=>rows(open,0,null,'forward',true),/not closed/);
assert.throws(()=>rows(open,2,0,'forward',false),/reachable/);
assert.throws(()=>rows(closed,1,1,'forward',false),/distinct end/);
console.log('Route selection: seam rotation, direction, explicit open endpoints and wrap rules passed.');

}
