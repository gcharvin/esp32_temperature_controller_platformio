# /// script
# requires-python = ">=3.11,<3.14"
# dependencies = ["cadquery==2.6.1", "pillow>=11,<12"]
# ///
"""Parametric solid mock-up, STEP/STL/glTF exports and actual-geometry previews.

Run: uv run hardware/mechanical/build_enclosure.py
Coordinates: x left->right, y front->rear, z bottom->top, millimetres.
STEP is a named solid assembly, not a native Fusion feature timeline.
"""
import json
from pathlib import Path
import math
import sys
import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
P=json.loads((HERE/'enclosure_parameters.json').read_text(encoding='utf-8'))
LAYOUT=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
OUT=HERE/'exports';OUT.mkdir(exist_ok=True)
W,H,D,T=[P['case_mm'][key] for key in ('width','height','depth','wall')]
PARTS=[]
LCD_DETAIL=[]
FRAME_CHECKS=[]
FRAME_GEOMETRIES=[]
RELEASED_PRINT_SHAPES={}
PRINT_NAMES={
 '01_Shell_open_front_and_rear':'01_Coque_ouverte',
 '02_Removable_front_LCD_A':'02_Facade_supports_LCD_A',
 '03_B_removable_carrier':'03_Berceau_B',
 '04_Rear_panel_with_latches':'04_Panneau_arriere',
 '05_XL4015_removable_carrier':'05_Berceau_XL4015'}
COLORS={'shell':(.78,.82,.85),'rear':(.54,.64,.70),'tray':(.95,.64,.25),
        'pcb':(.10,.48,.34),'module':(.20,.24,.29),'connector':(.92,.90,.79),
        'chip':(.18,.20,.22),'lcd':(.08,.19,.26),'metal':(.68,.71,.74),
        'wire':(.22,.46,.82),'window':(.17,.51,.69),'trimmer':(.12,.30,.80)}

def box(x,y,z,w,d,h):
    assert min(w,d,h)>0
    return cq.Workplane('XY').box(w,d,h,centered=(False,False,False)).translate((x,y,z))

def cyl(x,y,z,r,length):
    return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x,y,z),cq.Vector(0,1,0)))

def cyl_x(x,y,z,r,length):
    return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x,y,z),cq.Vector(1,0,0)))

def fuse(shapes):
    result=shapes[0]
    for shape in shapes[1:]:result=result.union(shape)
    return result.clean()

def add(name,shape,color,group,physical=True):
    if not shape.val().isValid():raise ValueError('Invalid '+name)
    PARTS.append({'name':name,'shape':shape,'color':COLORS.get(color,color),'group':group,'physical':physical})
    return shape

def pcb_frame(x,zbottom,width,height,face,open_upper=False,groove_capture=3):
    """Continuous lower groove, side guides and two outward-releasing side clips.

    Insert lower board edge at a slight tilt, then pivot toward the front support.
    No lateral insertion through a closed case wall is required.
    """
    thick=P['pcb_thickness_mm'];cl=P['clip_mm'];back=face-thick
    gap=cl['slot_clearance']/2;top=zbottom+height
    front=box(x-4,back-3,zbottom-4,width+8,3-gap,height+8)
    front=front.cut(box(x,back-3.1,zbottom+4,width,3.2,height-8))
    if open_upper:
        front=front.cut(box(x,back-3.1,top-4,width,3.2,8.2))
    pieces=[front]
    for side_x in (x,x+width-1.5):
        pieces.append(box(side_x,back-3,zbottom+4,1.5,3-gap,height-4))
    # Continuous C-section; capture is reduced for the unmeasured buck edge.
    pieces.append(box(x-2,back-.4,zbottom-2,width+4,thick+1.8,2-gap))
    pieces.append(box(x-2,face+gap,zbottom-2,width+4,1.4-gap,2+groove_capture))
    # Upper locating web: rear is open to permit pivot-and-snap insertion.
    pieces.append(box(x-2,back-.4,top+gap,width+4,thick+1.8,2-gap))
    # Continuous lateral guide walls; clipping supplies the rear jaws.
    for side_x in (x-2,x+width+gap):
        guide=box(side_x,back-.4,zbottom,2-gap,thick+2,height)
        guide=guide.cut(box(side_x-.1,back-.5,top-12,2.2,thick+2.2,5))
        pieces.append(guide)
    # Springs stand behind the guide walls so they remain free to bend sideways.
    fixed_frame=fuse(pieces)
    free_nominal=[];free_released=[];roots=[]
    for right in (False,True):
        root_z=top-25
        root=box(x-3,back-.5,root_z,3-gap,thick+5.1,2)
        arm=box(x-3,face+2.2,top-24,cl['arm_thickness'],2.4,cl['arm_length'])
        hook=(cq.Workplane('XY').polyline([
            (x-3,face+gap),(x+cl['engagement'],face+gap),
            (x-1.8,face+4.6),(x-3,face+4.6)]).close().extrude(3)
            .translate((0,0,top-11)))
        clip=fuse([root,arm,hook])
        root_overlap=root.val().intersect(fixed_frame.val()).Volume()
        free_arm_overlap=arm.union(hook).val().intersect(fixed_frame.val()).Volume()
        assert root_overlap>1e-4, 'Clip root is not attached to frame'
        assert free_arm_overlap<1e-4, 'Clip arm/hook is welded to a fixed guide'
        FRAME_CHECKS.append({'board_origin_x_z_mm':[x,zbottom],'side':'right' if right else 'left',
                             'root_connected':True,'free_arm_overlap_mm3':round(free_arm_overlap,6)})
        free=arm.union(hook)
        if right:
            clip=clip.mirror('YZ',(x+width/2,0,0))
            root=root.mirror('YZ',(x+width/2,0,0))
            free=free.mirror('YZ',(x+width/2,0,0))
        free_nominal.append(free);roots.append(root)
        free_released.append(free.translate((2.4 if right else -2.4,0,0)))
        pieces.append(clip)
    result=fuse(pieces)
    FRAME_GEOMETRIES.append({'x':x,'z':zbottom,'width':width,'height':height,'face':face,
        'nominal':result,'released':fuse([fixed_frame]+roots+free_released),
        'free_nominal':free_nominal,'free_released':free_released,'roots':roots})
    return result


def frame_bridges(x,zbottom,width,height,face):
    back=face-P['pcb_thickness_mm']
    pieces=[]
    for z in (zbottom-4,zbottom+height+1):
        pieces.append(box(T-.5,back-2.7,z,x-4+1-(T-.5),2.4,3))
        pieces.append(box(x+width+3,back-2.7,z,W-T+.5-(x+width+3),2.4,3))
    return pieces

def main(validate=True):
    front=P['rear_panel_mm']['thickness'];rear_y=D-front
    # Open-ended tube. Front is a separate, removable equipped faceplate.
    shell=box(0,T,0,W,rear_y-T,H).cut(box(T,T-.1,T,W-2*T,rear_y-T+.2,H-2*T))
    facade=box(0,0,0,W,T,H)
    win=P['lcd_window_mm']
    facade=facade.cut(box(win['center_x']-win['width']/2,-.1,win['center_z']-win['height']/2,
                         win['width'],T+.2,win['height']))
    enc=LAYOUT['assembly']['encoder_center_x_y_mm'];encz=H-enc[1]
    facade=facade.cut(cyl(enc[0],-.1,encz,3.6,T+.2))
    # Locating lip with clearance. Two side release windows for each end panel.
    rc=P['rear_panel_mm']['lip_clearance']
    lip=box(T+rc,T,T+rc,W-2*(T+rc),3,H-2*(T+rc))
    lip=lip.cut(box(T+rc+1.6,T-.1,T+rc+1.6,W-2*(T+rc+1.6),3.2,H-2*(T+rc+1.6)))
    facade=facade.union(lip)
    for start,length in ((9,9),(98,8)):
        for side_x in (-.1,W-T-.1):shell=shell.cut(box(side_x,start,45.5,T+.2,length,9))
    fa=box(3.35,3,46,1.2,14,8)
    fh=cq.Workplane('XY').polyline([(3.35,17),(2.1,14),(2.1,11),(4.55,11),(4.55,17)]).close().extrude(8).translate((0,0,46))
    fl=fa.union(fh);fr=fl.mirror('YZ').translate((W,0,0))
    front_latches=[fl,fr]
    facade=fuse([facade,fl,fr])
    usb=P['usb_service'];uy=usb['center_depth_mm'];uz=usb['center_z_mm']
    assert usb['side']=='right'
    shell=shell.cut(cyl_x(W-T-.1,uy,uz,usb['hole_diameter_mm']/2,T+.2))
    # Both holders are part of the FRONT piece, not the tube.
    lcd_frame=pcb_frame(8,28,98,60,15.1)
    ax,ay,aface=LAYOUT['assembly']['A_board_origin_x_y_d_mm'];az=H-ay-75
    a_frame=pcb_frame(ax,az,110,75,aface,open_upper=True)
    posts=[]
    for px,pz,pw,ph,pface in [(8,28,98,60,15.1),(ax,az,110,75,aface)]:
        back=pface-P['pcb_thickness_mm']
        for sx in (px-4,px+pw+1):
            for sz in (pz-4,pz+ph+1):posts.append(box(sx,T-.5,sz,3,back-.3-(T-.5),3))
    bare_front=facade
    facade=fuse([facade,lcd_frame,a_frame]+posts)
    LCD_DETAIL.extend([
        {'name':'Front_support_piece','shape':bare_front,'color':COLORS['shell'],'group':'lcd_detail'},
        {'name':'LCD_mount_integral_with_front','shape':fuse([lcd_frame]+posts[:4]),'color':COLORS['tray'],'group':'lcd_detail'}])
    RELEASED_PRINT_SHAPES['front']=facade.cut(fl).cut(fr).union(fl.translate((1.2,0,0))).union(fr.translate((-1.2,0,0)))
    # B support is a separate removable carrier; slide direction is +y.
    bx,by,bface=LAYOUT['assembly']['B_board_origin_x_y_d_mm'];bz=H-by-65
    tray=pcb_frame(bx,bz,90,65,bface)
    tray=fuse([tray,box(bx-7,bface-3,bz+23,4,21,10),box(bx+93,bface-3,bz+23,4,21,10)])
    gap=P['tray_clearance_mm'];fmin=bz-4;fmax=bz+65+4
    # Lateral guide ledges attach to the side walls and clear the rear lip.
    for z in (fmin-gap-2,fmax+gap):
        shell=shell.union(box(T-.5,78,z,bx+4-(T-.5),29,2))
        shell=shell.union(box(bx+86,78,z,W-T+.5-(bx+86),29,2))
    # Front stops, only at the carrier sides; board remains freely extractible.
    for x in (bx-4,bx+86):shell=shell.union(box(x,77.5,fmin-gap-.3,8,2.9,4+gap+.3))
    # Independent buck cassette in the left bay, sliding out rearward.
    buck=P['buck'];kx,kz=buck['pcb_origin_x_z_mm'];kw,kh,kt=buck['board_mm'];kf=buck['pcb_face_depth_mm']
    assert kt==P['pcb_thickness_mm']
    buck_frame=pcb_frame(kx,kz,kw,kh,kf,groove_capture=buck['lower_groove_capture_mm'])
    buck_frame=buck_frame.union(box(4,83,33,4,20,10))
    for gz in (15.75,77.25):shell=shell.union(box(2.5,78,gz,37.55,24.7,2))
    shell=shell.union(box(38.55,78,15.75,1.5,29,63.5))
    shell=shell.union(box(2.5,75.5,18,7,2.9,4))
    shell=shell.clean()
    # Rear panel: integral locating lip and two cantilever side latches.
    rc=P['rear_panel_mm']['lip_clearance'];ld=P['rear_panel_mm']['lip_depth']
    panel=box(0,rear_y,0,W,front,H)
    lip=box(T+rc,rear_y-ld,T+rc,W-2*(T+rc),ld,H-2*(T+rc))
    lip=lip.cut(box(T+rc+1.6,rear_y-ld-.1,T+rc+1.6,W-2*(T+rc+1.6),ld+.2,H-2*(T+rc+1.6)))
    arm=box(3.35,98,46,1.2,14,8)
    hook=cq.Workplane('XY').polyline([(3.35,98),(1.8,101),(1.8,104),(4.55,104),(4.55,98)]).close().extrude(8).translate((0,0,46))
    latch=arm.union(hook)
    panel=fuse([panel,lip,latch,latch.mirror('YZ').translate((W,0,0))])
    # Soft stop behind the carrier's right handle, gap at rest.
    panel=panel.union(box(bx+93,103.3,bz+24,4,rear_y-103.3,8))
    panel=panel.union(box(4,103.3,34,4,rear_y-103.3,8))
    for port in P['bulkheads']:
        panel=panel.cut(cyl(port['center_x_mm'],rear_y-ld-.1,port['center_z_mm'],
                            port['hole_diameter_mm']/2,front+ld+.2))
    RELEASED_PRINT_SHAPES['rear']=panel.cut(latch).cut(latch.mirror('YZ').translate((W,0,0))).union(latch.translate((1.5,0,0))).union(latch.mirror('YZ').translate((W-1.5,0,0)))
    add('05_XL4015_removable_carrier',buck_frame,'tray','buck')
    add('01_Shell_open_front_and_rear',shell,'shell','shell')
    add('02_Removable_front_LCD_A',facade,'shell','front')
    add('03_B_removable_carrier',tray,'tray','B')
    add('04_Rear_panel_with_latches',panel.clean(),'rear','rear')
    # Boards and all component envelopes come from the current placement study.
    for board in LAYOUT['boards']:
        name=board['name'];is_a=name=='pcb_a_display';group='A' if is_a else 'B'
        x,y,face=LAYOUT['assembly'][group+'_board_origin_x_y_d_mm']
        bw,bh,bt=board['board_mm'];bottom=H-y-bh
        add('PCB_'+group,box(x,face-bt,bottom,bw,bt,bh),'pcb',group)
        for row in board['components']:
            rx,ry,rw,rh=row['bounds_mm'];ref=row['reference']
            if is_a and ref=='U1':continue
            if ref in ('J1','J6','J7'):height=9.8;color='connector'
            elif row['footprint'] is None:
                height=20 if ref=='U1' else (10 if ref.startswith('J') else 8)
                color='module' if ref=='U1' else 'connector'
            else:height=1.5;color='chip'
            height=row.get('height_mm',height)
            add(group+'_'+ref+'_ENVELOPE',box(x+rx,face,H-y-ry-rh,rw,height,rh),color,group)
    # Purchased DevKit: two socket strips, PCB, RF shield and USB-C envelope.
    for z in (34.03,59.43):add('A_DEVKIT_SOCKET_'+str(z),box(20.95,45,z,38.1,8.5,2.54),'connector','A')
    add('A_DIYMORE_PCB_54x27',box(13,53.5,34.5,54,1.6,27),'pcb','A')
    add('A_DIYMORE_RF_SHIELD',box(21,55.1,36,23,3,24),'metal','A')
    add('A_DIYMORE_CH340',box(49,55.1,43,8,2,10),'chip','A')
    add('A_DIYMORE_USB_C',box(65,55.1,43.5,4,3,9),'metal','A')
    # XL4015 rotated in-plane to fit the independent left bay. Component
    # footprints are planning envelopes; a 2mm edge service band is assumed.
    add('XL4015_PCB',box(kx,kf-kt,kz,kw,kt,kh),'pcb','buck')
    buck_body=box(10,83,24,22.3,12.4,47)
    for z in (24,63):buck_body=buck_body.cut(box(9.9,82.9,z-.1,11.2,12.6,8.2))
    for x in (23,28):buck_body=buck_body.cut(box(x,89,43,4,6.5,6))
    add('XL4015_COMPONENT_ENVELOPE',buck_body,'module','buck')
    for z in (24,63):add('XL4015_TERMINAL_'+str(z),box(10,83,z,11,10,8),'connector','buck')
    for x in (23,28):add('XL4015_TRIMMER_'+str(x),box(x,89,43,4,6.4,6),'trimmer','buck')
    # LCD simplified reference assembly. Front optical opening is independent.
    add('LCD_PCB_98x60_TBD',box(8,13.5,28,98,1.6,60),'pcb','LCD')
    add('LCD_front_bezel_TBD',box(15,3.4,40,84,10.1,36),'lcd','LCD')
    add('LCD_backpack_TBD',box(51,15.1,34,45,12.9,20),'module','LCD')
    add('LCD_visible_screen',box(18.5,3.3,45.4,77,.1,25.2),'window','LCD')
    # Encoder shaft crosses its panel hole. Its body is a planning envelope.
    add('Encoder_body_TBD',box(enc[0]-8,3.2,encz-8,16,12,16),'module','LCD')
    add('Encoder_shaft',cyl(enc[0],-12,encz,3.2,15.2),'metal','LCD')
    add('Encoder_knob_TBD',cyl(enc[0],-17,encz,10,11),'module','LCD')
    # Commercial round panel USB extension. Not a second USB/UART converter.
    usb_port=fuse([
        cyl_x(W-21,uy,uz,usb['body_diameter_mm']/2,21),
        cyl_x(W-T-3,uy,uz,usb['flange_diameter_mm']/2,3),
        cyl_x(W,uy,uz,usb['flange_diameter_mm']/2,usb['outside_depth_mm']),
        cyl_x(W-usb['inside_depth_mm'],uy,uz,4,usb['inside_depth_mm']-21)])
    # Rectangular mouth is indicative, not a supplier-validated USB-C profile.
    usb_port=usb_port.cut(box(W-T-.1,uy-4.5,uz-1.75,usb['outside_depth_mm']+T+.2,9,3.5))
    add('USB_C_PANEL_DATA_EXTENSION_TBD',usb_port,'module','USB')
    add('USB_C_DEVKIT_MALE_PLUG_TBD',box(*usb['devkit_plug_box_mm']),'metal','USB')
    usb_path=[(127,64,30),(115,65,34),(96,65,48),(83,57.2,48)]
    usb_segments=[]
    for start,end in zip(usb_path,usb_path[1:]):
        vector=cq.Vector(*end)-cq.Vector(*start)
        usb_segments.append(cq.Workplane(obj=cq.Solid.makeCylinder(1.5,vector.Length,cq.Vector(*start),vector.normalized())))
    add('USB_INTERNAL_DATA_CABLE_ROUTING_ONLY',fuse(usb_segments),'wire','USB',False)
    for port in P['bulkheads']:
        x,z=port['center_x_mm'],port['center_z_mm'];radius=port['body_diameter_mm']/2
        stem_r=6;inside=port['inside_depth_mm'];outside=port['outside_depth_mm']
        body=fuse([cyl(x,D-inside,z,stem_r,inside+3),cyl(x,rear_y-3,z,radius,3),
                   cyl(x,D,z,radius,outside)])
        bore=port['cable_diameter_mm']/2+.4
        body=body.cut(cyl(x,D-inside-.1,z,bore,inside+outside+.2))
        add('BULKHEAD_'+port['name']+'_TBD',body,'metal' if 'PROBE' in port['name'] else 'module','rear')
        add('CABLE_'+port['name'],cyl(x,D-inside,z,port['cable_diameter_mm']/2,inside+outside+12),'wire','rear',False)
    # Three A/B bundle paths, only indicative routing, not minimum bend radii.
    paths=[[(102,54.8,76),(138,62,76),(138,100,76),(55,100,29),(55,94.8,29)],
           [(102,54.8,60),(138,64,60),(138,101,60),(74,101,29),(74,94.8,29)],
           [(96,54.8,43),(138,66,55),(138,102,55),(96,102,29),(96,94.8,29)]]
    for i,path in enumerate(paths,1):
        segments=[]
        for start,end in zip(path,path[1:]):
            v=cq.Vector(*end)-cq.Vector(*start)
            segments.append(cq.Workplane(obj=cq.Solid.makeCylinder(1.1,v.Length,cq.Vector(*start),v.normalized())))
        add(f'BUNDLE_AB_{i}_SCHEMATIC',fuse(segments),'wire','wire',False)
    if validate:validate_export()
    render_previews()

def _shape(value):
    return value.val() if isinstance(value,cq.Workplane) else value


def _overlap(a,b):
    a=_shape(a);b=_shape(b);aa=a.BoundingBox();bb=b.BoundingBox()
    if (aa.xmax<=bb.xmin+1e-7 or bb.xmax<=aa.xmin+1e-7 or
        aa.ymax<=bb.ymin+1e-7 or bb.ymax<=aa.ymin+1e-7 or
        aa.zmax<=bb.zmin+1e-7 or bb.zmax<=aa.zmin+1e-7):return 0.0
    return a.intersect(b).Volume()


def translation_prisms(shape,delta):
    """Exact translational sweep: start solid plus prisms of planar boundary faces.

    Cylindrical faces parallel to the translation are invariant laterally and
    their end faces provide the sweep. Other curved face orientations reject.
    """
    shape=_shape(shape);vector=cq.Vector(*delta);direction=vector.normalized()
    result=[shape]
    for face in shape.Faces():
        dot=abs(face.normalAt().dot(direction))
        if face.geomType()!='PLANE':
            if dot>1e-6:raise ValueError('Unsupported curved sweep face: '+face.geomType())
            continue
        if dot<1e-7:continue
        prism=cq.Solid.extrudeLinear(face.outerWire(),face.innerWires(),vector)
        if prism.Volume()>1e-8:result.append(prism)
    return result


def verify_assembly_paths():
    byname={p['name']:p for p in PARTS}
    facade=byname['02_Removable_front_LCD_A']['shape']
    tube=byname['01_Shell_open_front_and_rear']['shape']
    lcd=[p for p in PARTS if p['name'].startswith('LCD_')]
    encoder=[p for p in PARTS if p['name'].startswith('Encoder_')]
    a_parts=[p for p in PARTS if p['group']=='A']
    b_parts=[p for p in PARTS if p['group']=='B']
    usb_fixed=[p for p in PARTS if p['name']=='USB_C_PANEL_DATA_EXTENSION_TBD']
    assert len(usb_fixed)==1
    tests=[];failures=[]
    def collision(name,payload,obstacles):
        for item in payload:
            for obstacle in obstacles:
                volume=_overlap(item,obstacle)
                if volume>1e-4:failures.append({'stage':name,'overlap_mm3':round(volume,6)})
    def shift_shapes(shapes,vector):return [_shape(s).translate(vector) for s in shapes]
    def translating(name,shapes,delta,obstacles):
        count=0
        for shape in shapes:
            prisms=translation_prisms(shape,delta);count+=len(prisms)
            collision(name,prisms,obstacles)
        tests.append({'stage':name,'method':'planar-face translational swept volumes',
                      'translation_mm':delta,'prism_count':count,'reversible':True})
    def released(holder,frame):
        result=holder
        for free in frame['free_nominal']:result=result.cut(free)
        return fuse([result]+frame['roots']+frame['free_released'])
    def board_loading(tag,parts,frame,holder,extra,via_a=False):
        shapes=[p['shape'] for p in parts]
        pivot=(0,frame['face']-P['pcb_thickness_mm']/2,frame['z'])
        axis=(1,pivot[1],pivot[2]);angle=4.0
        tilted=[s.rotate(pivot,axis,-angle) for s in shapes]
        obstacles=[released(holder,frame)]+[_shape(p['shape']) for p in extra]
        if via_a:
            # Empty A holder aperture is 107 mm wide and open under its upper web.
            staged=shift_shapes(tilted,(4,70,-1))
            translating(tag+': through empty A holder',staged,(0,-64,0),obstacles)
            staged=shift_shapes(tilted,(4,6,-1))
            translating(tag+': align laterally',staged,(-4,0,0),obstacles)
            staged=shift_shapes(tilted,(0,6,-1))
            translating(tag+': raise above bottom groove lip',staged,(0,0,4.5),obstacles)
            offset=6
        else:offset=40
        staged=shift_shapes(tilted,(0,offset,3.5))
        translating(tag+': approach above groove lip',staged,(0,-offset,0),obstacles)
        staged=shift_shapes(tilted,(0,0,3.5))
        translating(tag+': lower into groove',staged,(0,0,-3.5),obstacles)
        for step in range(41):
            theta=angle*(40-step)/40
            pose=[s.rotate(pivot,axis,-theta) for s in shapes]
            collision(tag+': pivot '+str(round(theta,1))+' deg',pose,obstacles)
        tests.append({'stage':tag+': pivot and release clips','method':'BRep poses at 0.1 degree intervals',
                      'angle_deg':[4,0],'clips_held_outward_mm':2.4,
                      'note':'Free arm/hook translated as release envelope; no elastic/FEM claim','reversible':True})
    lcd_frame,a_frame,b_frame,buck_frame=FRAME_GEOMETRIES
    board_loading('1 LCD outside tube',lcd,lcd_frame,facade,[],True)
    board_loading('2 A outside tube',a_parts,a_frame,facade,lcd+encoder)
    board_loading('3 B outside tube',[p for p in b_parts if p['name'] not in PRINT_NAMES],b_frame,byname['03_B_removable_carrier']['shape'],[])
    buck_parts=[p for p in PARTS if p['group']=='buck']
    board_loading('3b buck outside tube',[p for p in buck_parts if p['name'] not in PRINT_NAMES],buck_frame,byname['05_XL4015_removable_carrier']['shape'],[])
    # Front latch hooks are held inward; male USB and flexible bundles disconnected.
    equipped_front=[RELEASED_PRINT_SHAPES['front']]+[p['shape'] for p in a_parts+lcd+encoder]
    translating('4 equipped front enters from front',shift_shapes(equipped_front,(0,-100,0)),(0,100,0),
                [tube]+[p['shape'] for p in usb_fixed])
    front_at_rest=[facade]+[p['shape'] for p in a_parts+lcd+encoder]
    usb_plug=byname['USB_C_DEVKIT_MALE_PLUG_TBD']['shape']
    translating('5 USB male plug insertion after front fitted',[usb_plug.translate((15,0,0))],(-15,0,0),
                [tube]+front_at_rest+[p['shape'] for p in usb_fixed])
    translating('6 B carrier extraction / reverse insertion',[p['shape'] for p in b_parts],(0,50,0),
                [tube]+front_at_rest+[p['shape'] for p in usb_fixed]+[usb_plug]+[p['shape'] for p in buck_parts])
    translating('6b independent buck extraction',[p['shape'] for p in buck_parts],(0,60,0),[tube]+front_at_rest+[p['shape'] for p in usb_fixed]+[usb_plug]+[p['shape'] for p in b_parts])
    rear_parts=[RELEASED_PRINT_SHAPES['rear']]+[p['shape'] for p in PARTS if p['group']=='rear' and p['physical'] and not p['name'].startswith('04_')]
    translating('7 rear panel withdrawal / reverse insertion',rear_parts,(0,60,0),
                [tube]+front_at_rest+[p['shape'] for p in usb_fixed]+[usb_plug]+[p['shape'] for p in b_parts+buck_parts])
    # Tool service volumes after rear panel removal: straight rear approach.
    tools=[]
    for tx in P['buck']['terminal_screw_x_mm']:
        for tz in P['buck']['terminal_screw_z_mm']:tools.append(cyl(tx,96,tz,3,50))
    for tx in P['buck']['pot_screw_x_mm']:tools.append(cyl(tx,96,46,3,50))
    collision('8 buck terminal and trimmer screwdriver corridors',tools,[p['shape'] for p in PARTS if p['physical'] and p['group'] not in ('rear','wire')])
    tests.append({'stage':'8 buck screwdriver service','shaft_diameter_mm':6,'rear_panel_removed':True,'method':'BRep intersection of reserved tool cylinders','terminal_and_trimmer_positions':'approximate, verify purchased module'})
    # Free corridors ABOVE the terminal/housing envelopes. Connector mating
    # detail is deliberately not asserted by these simplified component boxes.
    bspec=next(b for b in LAYOUT['boards'] if b['name']=='pcb_b_temperature')
    bx,by,bface=LAYOUT['assembly']['B_board_origin_x_y_d_mm']
    tool_corridors=[];plug_corridors=[]
    for row in bspec['components']:
        ref=row['reference'];rx,ry,rw,rh=row['bounds_mm']
        y0=bface+row['height_mm']+.2
        if ref in ('J2','J3','J8'):
            px,py=row['origin_mm'];angle=math.radians(row['rotation_deg'])
            for dx in (0,5.08):
                tool_corridors.append(cyl(bx+px+dx*math.cos(angle),y0,H-by-py+dx*math.sin(angle),3,50))
        elif ref in ('J1','J4','J6','J7','J9'):
            plug_corridors.append(box(bx+rx-1,y0,H-by-ry-rh-1,rw+2,35,rh+2))
    service_obstacles=[p['shape'] for p in PARTS if p['physical'] and p['group'] not in ('rear','wire')]
    collision('9 B terminal screwdriver corridors',tool_corridors,service_obstacles)
    collision('9 B JST axial service corridors',plug_corridors,service_obstacles)
    tests.append({'stage':'9 B connector service','rear_panel_removed':True,'shaft_diameter_mm':6,
                  'JST_rear_corridor_length_mm':35,'terminal_refs':['J2','J3','J8'],
                  'JST_refs':['J1','J4','J6','J7','J9'],
                  'method':'BRep intersection above connector envelopes; actual contacts and side wire entries unqualified'})
    result={'tests':tests,'failures':failures,
            'reverse_order':'rear off; B or buck independently disconnected/extracted; USB male unplugged before front released/pulled; A then LCD unclipped',
            'limits':'Rigid-body geometry only. Rotations sampled at 0.1 deg. Latches represented as released envelopes. Cable slack, fingers, flexure and print tolerances require physical validation.'}
    (OUT/'assembly_sequence_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if failures:raise AssertionError(failures[:25])
    print('Assembly paths verified:',len(tests),'stages; rigid-body sweeps and sampled pivots.')
    return result


def validate_export():
    assembled=cq.Assembly(name='Laboratory_controller_fit_study_05_XL4015_Diymore')
    exploded=cq.Assembly(name='Exploded_fit_study_05_XL4015_Diymore')
    printed=cq.Assembly(name='Boitier_5_pieces_a_imprimer')
    printed_exploded=cq.Assembly(name='Boitier_5_pieces_vue_eclatee')
    print_names=PRINT_NAMES
    groups={key:cq.Assembly(name=label) for key,label in [
        ('plastic','01_Pieces_imprimees'),('A','02_Carte_A_reference'),
        ('B','03_Carte_B_reference'),('buck','07_XL4015_reference'),('LCD','04_Afficheur_encodeur_reference'),
        ('external','05_Passe_paroi_et_faisceaux_reference'),('USB','06_USB_maintenance_reference')]}
    partsdir=OUT/'parts_v05';partsdir.mkdir(exist_ok=True)
    summary={'status':'unqualified fit mock-up, not production design','parts':[],'collisions_mm3':[],
             'coordinates':'x left-right, y front-rear, z bottom-top (mm)',
             'case_mm':P['case_mm'],'bulkheads':P['bulkheads'],'usb_service':P['usb_service'],'retention':P['retention'],
             'USB_electrical_VBUS_isolation_verified':False,'frame_clip_checks':FRAME_CHECKS}
    for part in PARTS:
        s=part['shape'];name=part['name'];vol=s.val().Volume()
        assert vol>0 and s.val().isValid(),name
        group='plastic' if name in print_names else part['group'] if part['group'] in ('A','B','LCD','USB','buck') else 'external'
        groups[group].add(s,name=name,color=cq.Color(*part['color']))
        shift=-85 if part['group'] in ('front','A','LCD') else 60 if part['group']=='buck' else 50 if part['group']=='B' else (85 if part['group']=='rear' else 0)
        if part['group']!='wire':exploded.add(s.translate((0,shift,0)),name=name,color=cq.Color(*part['color']))
        summary['parts'].append({'name':name,'volume_mm3':round(vol,3),'solids':len(s.val().Solids()),'group':part['group']})
        if name in PRINT_NAMES:
            assert len(s.val().Solids())==1,(name,'print part is disconnected',[(round(a.Volume(),2),[round(v,2) for v in (a.BoundingBox().xmin,a.BoundingBox().ymin,a.BoundingBox().zmin,a.BoundingBox().xmax,a.BoundingBox().ymax,a.BoundingBox().zmax)]) for a in s.val().Solids()])
            cq.exporters.export(s,str(partsdir/(name+'.step')))
            cq.exporters.export(s,str(partsdir/(name+'.stl')),tolerance=.15,angularTolerance=.15)
            printed.add(s,name=print_names[name],color=cq.Color(*part['color']))
            printed_exploded.add(s.translate((0,shift,0)),name=print_names[name],color=cq.Color(*part['color']))
    # Exact BRep intersection of plastics, boards and component envelopes.
    # Adjacent encoder shaft/knob/body and nested LCD screen are intentional mates.
    plastic=[p for p in PARTS if p['name'] in PRINT_NAMES]
    others=[p for p in PARTS if p['physical'] and p not in plastic]
    for a in plastic:
        for b in plastic+others:
            if a is b:continue
            if b in plastic and a['name']>b['name']:continue
            overlap=a['shape'].val().intersect(b['shape'].val()).Volume()
            if overlap>1e-4:summary['collisions_mm3'].append([a['name'],b['name'],round(overlap,5)])
    for a,b in [('PCB_A','PCB_B')]:
        sa=next(p['shape'] for p in PARTS if p['name']==a)
        sb=next(p['shape'] for p in PARTS if p['name']==b)
        assert sa.val().intersect(sb.val()).Volume()<1e-5
    b_parts=[p for p in PARTS if p['group']=='B' and p['physical']]
    for usb_part in [part for part in PARTS if part['group']=='USB' and part['physical']]:
        for other in [part for part in PARTS if part['group'] in ('A','B','LCD') and part['physical']]:
            overlap=usb_part['shape'].val().intersect(other['shape'].val()).Volume()
            if overlap>1e-4:summary['collisions_mm3'].append([usb_part['name'],other['name'],round(overlap,5)])
    b_envelope=cq.Compound.makeCompound([p['shape'].val() for p in b_parts]).BoundingBox()
    swept=box(b_envelope.xmin,b_envelope.ymin,b_envelope.zmin,
              b_envelope.xlen,b_envelope.ylen+50,b_envelope.zlen)
    # B and its carrier can translate 50 mm rearward after panel/cables removal.
    fixed=[p for p in PARTS if p['group'] in ('shell','front','A','LCD','USB','buck')]
    for obstacle in fixed:
        volume=swept.val().intersect(obstacle['shape'].val()).Volume()
        if volume>1e-4:summary['collisions_mm3'].append(['B_swept_bounding_box_0_to_50',obstacle['name'],round(volume,5)])
    for distance in (0,10,25,50):
        for moving in [p for p in PARTS if p['group']=='B' and p['physical']]:
            shape=moving['shape'].translate((0,distance,0))
            for obstacle in fixed:
                vol=shape.val().intersect(obstacle['shape'].val()).Volume()
                if vol>1e-4:summary['collisions_mm3'].append(['B_extraction_'+str(distance),moving['name'],obstacle['name'],round(vol,5)])
    (OUT/'validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    if summary['collisions_mm3']:raise AssertionError(summary['collisions_mm3'])
    summary['assembly_sequence']=verify_assembly_paths()
    for group in groups.values():assembled.add(group)
    assembled.export(str(OUT/'enclosure_assembled.step'))
    exploded.export(str(OUT/'enclosure_exploded.step'))
    assembled.export(str(OUT/'enclosure_assembled.glb'))
    printed.export(str(OUT/'enclosure_printed_parts.step'))
    printed_exploded.export(str(OUT/'enclosure_printed_parts_exploded.step'))
    imported_printed=cq.importers.importStep(str(OUT/'enclosure_printed_parts.step'))
    print_solids=imported_printed.val().Solids()
    assert len(print_solids)==5 and all(s.isValid() for s in print_solids)
    expected_volume=sum(part['shape'].val().Volume() for part in PARTS if part['name'] in print_names)
    assert math.isclose(imported_printed.val().Volume(),expected_volume,rel_tol=1e-8)
    (OUT/'enclosure_v05_xl4015_diymore.step').write_bytes((OUT/'enclosure_assembled.step').read_bytes())
    (OUT/'enclosure_v05_printed_parts.step').write_bytes((OUT/'enclosure_printed_parts.step').read_bytes())
    summary['printed_parts_STEP']={'file':'enclosure_printed_parts.step','components':list(print_names.values()),'solid_count':5,'one_connected_solid_per_print_part':True}
    # Round-trip the exchange format, including checks against accidental empty exports.
    reimport=cq.importers.importStep(str(OUT/'enclosure_assembled.step'))
    assert reimport.val().isValid() and len(reimport.val().Solids())>=len(PARTS)
    summary['STEP_round_trip_solids']=len(reimport.val().Solids())
    summary['B_extraction_samples_mm']=[0,10,25,50]
    summary['B_continuous_extraction_check']='Conservative swept bounding box clear from 0 to 50 mm, rear panel and all cables removed'
    summary['STEP_volume_mm3']=round(reimport.val().Volume(),3)
    summary['fusion_import_tested']=False
    (OUT/'validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print('STEP/STL/GLB exported; BRep collision and extraction samples passed.')

def render_previews():
    import vtk
    from PIL import Image,ImageDraw,ImageFont
    def actor(part,shift=0,opacity=1):
        verts,triangles=part['shape'].val().tessellate(.3,.2)
        points=vtk.vtkPoints()
        for v in verts:points.InsertNextPoint(v.x,v.y+shift,v.z)
        cells=vtk.vtkCellArray()
        for tri in triangles:
            cells.InsertNextCell(3)
            for idx in tri:cells.InsertCellPoint(idx)
        mesh=vtk.vtkPolyData();mesh.SetPoints(points);mesh.SetPolys(cells)
        normals=vtk.vtkPolyDataNormals();normals.SetInputData(mesh);normals.SetFeatureAngle(50)
        mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
        a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*part['color'])
        a.GetProperty().SetOpacity(opacity);a.GetProperty().SetSpecular(.2);a.GetProperty().SetSpecularPower(35)
        return a
    for mode,camera in [('assembled',(230,-260,175)),('exploded',(290,330,230)),('service',(260,400,185)),('buck',(100,240,140)),('front_module',(220,200,155)),('front_lcd',(190,160,140)),('rear',(72,400,50)),('clips',(175,245,145)),('lcd_mount',(185,155,140)),('usb',(340,-80,140))]:
        renderer=vtk.vtkRenderer();renderer.SetBackground(.94,.96,.97)
        scene=LCD_DETAIL+[p for p in PARTS if p['name'].startswith('LCD_')] if mode=='lcd_mount' else PARTS
        for part in scene:
            if mode=='clips' and part['group']!='B':continue
            if mode=='buck' and part['group']!='buck':continue
            if mode=='service' and (part['group'] in ('rear','wire') or not part['physical']):continue
            if mode=='front_module' and part['group'] not in ('front','A','LCD'):continue
            if mode=='front_lcd' and not (part['group']=='front' or part['name'].startswith('LCD_')):continue
            if mode=='exploded' and (part['group']=='wire' or not part['physical']):continue
            shift=(-85 if part['group'] in ('front','A','LCD') else 60 if part['group']=='buck' else 50 if part['group']=='B' else 85 if part['group']=='rear' else 0) if mode=='exploded' else 0
            alpha=.15 if mode in ('exploded','service') and part['group']=='shell' else 1
            renderer.AddActor(actor(part,shift,alpha))
        cam=renderer.GetActiveCamera();cam.SetPosition(*camera);cam.SetFocalPoint(85 if mode=='clips' else 72,12 if mode=='lcd_mount' else 85 if mode=='clips' else 90 if mode=='exploded' else 52,58 if mode=='lcd_mount' else 50)
        cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();renderer.ResetCamera();cam.Zoom(.85 if mode=='buck' else .90 if mode in ('front_lcd','front_module') else 1.0 if mode=='lcd_mount' else 1.12)
        window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1600,1100)
        window.SetMultiSamples(8);window.AddRenderer(renderer);window.Render()
        capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.Update()
        file=OUT/('preview_'+mode+'.png');writer=vtk.vtkPNGWriter();writer.SetFileName(str(file));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
        window.Finalize()
        im=Image.open(file).convert('RGB');draw=ImageDraw.Draw(im)
        try:font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',28);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',21)
        except OSError:font=small=ImageFont.load_default()
        titles={'assembled':'MAQUETTE 0.5 - facade / boitier assemble',
                'exploded':'MAQUETTE 0.5 - facade equipee / coque / B / buck / panneau arriere',
                'service':'ARRIERE OUVERT - carte B et XL4015 sur leurs berceaux independants',
                'buck':'XL4015 - berceau independant, rainure basse et deux clips lateraux',
                'front_module':'FACADE EQUIPEE - ecran puis carte A montes hors du boitier',
                'front_lcd':'ETAPE 1 - LCD monte, support A encore vide',
                'rear':'PANNEAU ARRIERE - passages de paroi provisoires',
                'clips':'BERCEAU B - rainure continue et deux clips lateraux liberables',
                'lcd_mount':'FIXATION DU LCD - vue depuis interieur, cartes A et B masquees',
                'usb':'USB LATERAL - acces prevu pour flash et monitoring serie'}
        draw.text((35,24),titles[mode],font=font,fill='#18313e')
        draw.text((35,1050),'Volumes de composants et connecteurs a confirmer - clips non qualifies - pas de fabrication finale',font=small,fill='#435966')
        if mode=='exploded':draw.text((35,75),'Facade vers avant ; B et buck independamment vers arriere, faisceaux debranches.',font=small,fill='#435966')
        if mode=='service':draw.text((35,75),'Coque rendue transparente pour inspection. Acces borniers et reglages depuis arriere.',font=small,fill='#435966')
        if mode=='buck':draw.text((35,75),'Dimensions module 51 x 26,3 x 14 mm ; details et marges de bord a mesurer.',font=small,fill='#435966')
        if mode=='front_module':draw.text((35,75),'Vue depuis arriere : A masque le LCD. Les deux supports appartiennent a la facade imprimee.',font=small,fill='#435966')
        if mode=='front_lcd':draw.text((35,75),'LCD passe dans le cadre A vide, puis se place dans sa rainure et ses clips. Poser A ensuite.',font=small,fill='#435966')
        if mode=='usb':
            draw.text((35,75),'Prise de panneau USB-C avec rallonge DATA vers la DevKit. Gestion du 5 V USB a valider.',font=small,fill='#435966')
            renderer.SetWorldPoint(W+4,P['usb_service']['center_depth_mm'],P['usb_service']['center_z_mm'],1)
            renderer.WorldToDisplay();px,py,_=renderer.GetDisplayPoint();py=1100-py
            draw.line([(1180,860),(px,py)],fill='#18313e',width=2)
            draw.ellipse((px-5,py-5,px+5,py+5),fill='#e78722')
            draw.text((1000,875),'USB : FLASH / SERIE',font=small,fill='#18313e')
        if mode=='lcd_mount':
            draw.text((35,75),'Orange : support imprime AVEC la facade (pas une piece supplementaire). Vert : PCB du LCD.',font=small,fill='#435966')
            callouts=[((106.8,17.5,78),(920,180),'Clips lateraux : liberables vers exterieur'),
                      ((75,16.1,29),(1000,900),'Rainure basse continue : porte le PCB'),
                      ((124,12,90),(45,260),'Ponts solidaires de la facade')]
            for point,(tx,ty),label in callouts:
                renderer.SetWorldPoint(*point,1);renderer.WorldToDisplay();px,py,_=renderer.GetDisplayPoint();py=1100-py
                bounds=draw.textbbox((tx,ty),label,font=small);draw.rectangle((bounds[0]-8,bounds[1]-7,bounds[2]+8,bounds[3]+7),fill='#f0f5f7')
                sx=(bounds[0]+bounds[2])/2;sy=bounds[3]+10 if ty<py else bounds[1]-10
                draw.line([(sx,sy),(px,py)],fill='#18313e',width=2)
                draw.ellipse((px-5,py-5,px+5,py+5),fill='#e78722')
                draw.text((tx,ty),label,font=small,fill='#18313e')
        if mode=='rear':
            draw.text((35,75),'Vue exterieure : sonde a gauche, alimentation et chauffage a droite',font=small,fill='#435966')
            for port in P['bulkheads']:
                renderer.SetWorldPoint(port['center_x_mm'],D+port['outside_depth_mm'],port['center_z_mm'],1)
                renderer.WorldToDisplay();px,py,_=renderer.GetDisplayPoint()
                label={'POWER_24V':'ENTREE 24 V','HEATER':'CHAUFFAGE','NTC_PROBE':'SONDE 2 CONTACTS'}[port['name']]
                draw.text((px-85,1100-py-90),label,font=small,fill='#18313e')
        im.save(file)
    print('Rendered actual CAD geometry previews.')

if __name__=='__main__':main(validate='--render-only' not in sys.argv)
