"""Native corrections of the staged local-router result; explicit topology only."""
import argparse,json,subprocess
from pathlib import Path
import pcbnew as p
import create_drafts as s
from integrate_buck_b import NAME,CLI,write_tree
from create_placement import pos,mm
from build_routing_boards import track

def finish(work):
 folder=work/NAME;path=folder/(NAME+'.kicad_pcb')
 board=p.LoadBoard(str(work/'pre_router.kicad_pcb'));board.SetFileName(str(path))
 assert p.ImportSpecctraSES(board,str(work/'B_single.ses'))
 p.SaveBoard(str(work/'router_import.kicad_pcb'),board)
 bt=s.parse((work/'router_import.kicad_pcb').read_text())
 old=(68.1019,86.0174);new=(68,88.5)
 for n in bt:
  if not isinstance(n,list):continue
  if n[0] in ['segment','via']:
   for key in (['at'] if n[0]=='via' else ['start','end']):
    pt=s.child(n,key)
    if tuple(round(float(v),4) for v in pt[1:3])==old:pt[1:3]=list(map(str,new))
 # Remove the open ground stub left at an old branch near Q1.
 bt=[n for n in bt if not (isinstance(n,list) and n[0]=='segment' and s.unquote(s.child(n,'net')[1])=='/PGND' and tuple(round(float(v),4) for v in s.child(n,'start')[1:3])==(137.0,82.5))]
 write_tree(path,bt)
 board=p.LoadBoard(str(path));nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
 # Join the router's two VIN islands on the underside, away from SW and FB.
 via=p.PCB_VIA(board);via.SetPosition(pos(28.1498,43.75));via.SetWidth(mm(1.6));via.SetDrill(mm(.8));via.SetLayerPair(p.F_Cu,p.B_Cu);via.SetNet(nets['/VIN_PROTECTED']);board.Add(via)
 track(board,nets['/VIN_PROTECTED'],[(28.1498,43.75),(28.1498,43.5),(12,43.5),(10.084,41.584),(10.084,36.9448)],1.2,p.B_Cu)
 track(board,nets['/GND'],[(8,33.5),(2.5,33.5),(2.5,49),(8.5,55),(15,55)],1.0,p.F_Cu)
 p.ZONE_FILLER(board).Fill(board.Zones());p.SaveBoard(str(path),board)
 # Match the footprint's fabrication/BOM exclusion: NT1 is copper, not a part.
 source=folder/(NAME+'.kicad_sch');tree=s.parse(source.read_text())
 for n in s.children(tree,'symbol'):
  props={s.unquote(v[1]):s.unquote(v[2]) for v in s.children(n,'property')}
  if props.get('Reference')=='NT1':s.child(n,'in_bom')[1]='no'
 write_tree(source,tree)
 subprocess.run([CLI,'pcb','drc','--schematic-parity','--format','json','--output',str(work/'finished_drc.json'),str(path)],check=True)

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--workdir',required=True,type=Path);a=parser.parse_args();finish(a.workdir.resolve())


