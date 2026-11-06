"""Removable BH1750 cassette and closed optical insert, in case coordinates.

Six print pieces including the existing four. Module dimensions are explicit
planning parameters, not a claim that an unidentified breakout was measured.
"""
import math
import cadquery as cq


def box(x,y,z,w,d,h):
    return cq.Workplane('XY').box(w,d,h,centered=(False,False,False)).translate((x,y,z))


def fuse(items):
    result=items[0]
    for item in items[1:]:result=result.union(item)
    return result.clean()


def build(facade,config):
    x,z=config['center_x_z_mm'];w,h,thick=config['module_pcb_mm']
    assert (w,h,thick)==(23,16,1.6),'Recheck clip geometry after changing module dimensions'
    membrane=config['membrane_mm'];assert .4<=membrane<=1.0
    # Insert from inside. The projecting stem enters the small front opening;
    # its wider flange seats in a rear rebate. No exterior hole remains open.
    facade=facade.cut(box(x-4.2,-.1,z-4.2,8.4,3.3,8.4))
    facade=facade.cut(box(x-6.2,2,z-6.2,12.4,1.3,12.4))
    skin=box(x-4,0,z-4,8,membrane,8)
    stem=box(x-4,membrane-.05,z-4,8,2.1-membrane,8).cut(box(x-3,membrane-.1,z-3,6,2.3,6))
    flange=box(x-6,2.05,z-6,12,1,12).cut(box(x-3,1.95,z-3,6,1.3,6))
    insert=fuse([skin,stem,flange])
    # Four face stops support the opaque cassette without touching the window.
    for xx in (x-14,x+12):
        for zz in (z-10.5,z+8.5):facade=facade.union(box(xx,2.8,zz,2,.4,2))
    # Two front-mounted cantilever latches capture cassette flange edges.
    mount_free=[];mount_released=[]
    for upper in (False,True):
        armz=z+11.1 if upper else z-12.1
        hookz=z+9.8 if upper else z-12.1
        root=box(x-10,2.5,armz,3,3.3,1)
        arm=box(x-7.5,3.3,armz,13.5,2.5,1)
        hook=box(x+4.5,4.8,hookz,1.5,1,2.3)
        free=arm.union(hook);facade=fuse([facade,root,free])
        mount_free.append(free);mount_released.append(free.translate((0,0,1.2 if upper else -1.2)))
    # Opaque removable cassette; its light well shields the sensor from LCD light.
    plate=box(x-14,3.2,z-10.5,28,1.3,21).cut(box(x-3.4,3.1,z-3.4,6.8,1.6,6.8))
    well=box(x-5,4.4,z-5,10,2.35,10).cut(box(x-3.4,4.3,z-3.4,6.8,2.6,6.8))
    fixed=[plate,well]
    for xx in (x-13.5,x+11.75):fixed.append(box(xx,4.4,z-8,1.75,4,16))
    for zz in (z-9.5,z+8.25):fixed.append(box(x-12,4.4,zz,24,4,1.25))
    for xx in (x-12,x+11):fixed.append(box(xx,6.5,z-6,1,.25,12))
    roots=[];pcb_free=[];pcb_released=[]
    for right in (False,True):
        root=box(x-13.5,4.4,z-7.5,1.75,6,2)
        arm=box(x-13.4,8.9,z-6,.9,1.5,10)
        hook=box(x-13.4,8.9,z+2,2.65,1,2)
        if right:
            root=root.mirror('YZ',(x,0,0));arm=arm.mirror('YZ',(x,0,0));hook=hook.mirror('YZ',(x,0,0))
        free=arm.union(hook);roots.append(root);pcb_free.append(free)
        pcb_released.append(free.translate((1.2 if right else -1.2,0,0)))
    carrier=fuse(fixed+roots+pcb_free)
    released_carrier=fuse(fixed+roots+pcb_released)
    pcb=box(x-w/2,7,z-h/2,w,thick,h)
    package=box(x-1.5,6.2,z-.8,3,.8,1.6)
    for label,shape in [('insert',insert),('carrier',carrier)]:
        assert shape.val().isValid() and len(shape.val().Solids())==1,label
    # A central probe intersects exactly the thin membrane, not an open hole.
    probe=box(x-.5,-.1,z-.5,1,5,1)
    assert math.isclose(insert.val().intersect(probe.val()).Volume(),membrane,abs_tol=1e-6)
    return dict(facade=facade,insert=insert,carrier=carrier,pcb=pcb,package=package,
        mount_free=mount_free,mount_released=mount_released,released_carrier=released_carrier,
        config=config,summary=dict(center_x_z_mm=[x,z],membrane_mm=membrane,central_optical_path_closed=True,
            optical_transmission_tested=False,module_dimensions_measured=False,
            membrane_back_to_package_front_mm=6.2-membrane,
            opaque_light_well=True,removable_cassette=True,one_solid_per_new_print=True))
