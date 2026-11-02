(function(root){
 function rows(route,start,end,direction,loop){
  const closed=route.definition.topology==='closed',body=route.center.length-(closed?1:0),step=direction==='forward'?1:-1;
  if(!Number.isInteger(start)||start<0||start>=body||!['forward','reverse'].includes(direction))throw Error('Invalid start or direction');
  if(loop){if(!closed)throw Error('Source route is not closed');return [...Array(body)].map((_,i)=>(start+step*i+body)%body).concat(start)}
  if(!Number.isInteger(end)||end<0||end>=body)throw Error('Choose a valid end row');
  const count=closed?((end-start)*step+body)%body:(end-start)*step;
  if(count<1)throw Error('Choose a distinct end reachable in this direction');
  return [...Array(count+1)].map((_,i)=>closed?(start+step*i+body)%body:start+step*i);
 }
 const api={rows};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.RouteSelection=api;
})(typeof globalThis!=='undefined'?globalThis:this);
