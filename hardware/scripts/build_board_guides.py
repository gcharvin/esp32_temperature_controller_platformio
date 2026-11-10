"""Illustrated French board guides from hash-checked native KiCad geometry.

Read-only for the electrical design. Run with the primary Python (ReportLab),
with pdftoppm on PATH. Outputs are explanatory drawings, not assembly exports.
"""
from pathlib import Path
import hashlib
import json
import math
import subprocess

from reportlab.graphics.shapes import Drawing, Rect, Line, Circle, Polygon, String, PolyLine
from reportlab.graphics import renderPDF, renderSVG
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A3, landscape

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'review/board_guides'
PDF = OUT / 'output/pdf/cartes_A_B_guide_annote.pdf'
W, H = 1700, 1200
C = dict(white='#ffffff', ink='#152f40', muted='#536b7c', bg='#f5f7fb', board='#dcece4',
         power='#c05b15', bus='#2372b7', ctrl='#7756b2', sensor='#168473',
         diag='#ad4678', gray='#647685')


def color(s): return HexColor(C.get(s, s))


class Page:
    def __init__(self):
        self.d = Drawing(W, H)
        self.box(0, 0, W, H, C['bg'], stroke=None)

    def box(self, x, y, w, h, fill='white', stroke='#dae2eb', radius=0, width=1):
        self.d.add(Rect(x, H-y-h, w, h, rx=radius, ry=radius,
                        fillColor=color(fill) if fill else None,
                        strokeColor=color(stroke) if stroke else None, strokeWidth=width))

    def line(self, pts, col='ink', width=2, dash=None):
        self.d.add(PolyLine([v for x, y in pts for v in (x, H-y)],
                           strokeColor=color(col), strokeWidth=width,
                           strokeDashArray=dash, fillColor=None))

    def circle(self, x, y, r, fill=None, stroke='ink', width=1):
        self.d.add(Circle(x, H-y, r, fillColor=color(fill) if fill else None,
                          strokeColor=color(stroke) if stroke else None, strokeWidth=width))

    def text(self, x, y, value, size=20, col='ink', bold=False, anchor='start'):
        self.d.add(String(x, H-y, value, fontName='Arial-Bold' if bold else 'Arial',
                          fontSize=size, fillColor=color(col), textAnchor=anchor))

    def arrow(self, pts, col, width=2.3):
        self.line(pts, col, width)
        x, y = pts[-1]; px, py = pts[-2]
        angle = math.atan2(y-py, x-px)
        vertices = [(x, y)] + [(x-11*math.cos(angle+a), y-11*math.sin(angle+a)) for a in (-.42, .42)]
        self.d.add(Polygon([v for xx, yy in vertices for v in (xx, H-yy)],
                           fillColor=color(col), strokeColor=None))
        self.circle(x, y, 6, None, col, 2)

    def card(self, box, title, lines, col, target, side='right', via=None):
        x, y, w, h = box
        assert 52 + len(lines)*23 <= h, (title, h)
        self.box(x, y, w, h, 'white', '#d7e0ea', 12)
        self.box(x, y+10, 5, h-20, col, None, 2)
        title_size=min(22,(w-34)*22/max(1,pdfmetrics.stringWidth(title,'Arial-Bold',22)))
        self.text(x+18, y+31, title, title_size, col, True)
        for i, line in enumerate(lines):
            assert pdfmetrics.stringWidth(line, 'Arial', 18) < w-32, (title, line)
            self.text(x+18, y+60+23*i, line, 18)
        if side == 'left': start = (x+w, y+h/2)
        elif side == 'top': start = (x+w/2, y+h)
        elif side == 'bottom': start = (x+w/2, y)
        else: start = (x, y+h/2)
        self.arrow([start, *(via or []), target], col)

    def heading(self, code, title, subtitle):
        self.box(50, 41, 58, 58, 'ink', None, 14)
        self.text(79, 82, code, 32, 'white', True, 'middle')
        self.text(128, 79, title, 36, 'ink', True)
        self.text(50, 119, subtitle, 21, 'muted')

    def footer(self, line1, line2):
        self.line([(50, 1130), (1650, 1130)], '#d4dde8', 1)
        self.text(50, 1158, line1, 18)
        self.text(50, 1186, line2, 16, 'muted')


def board(page, b, left, top, scale):
    width, height = b['size_mm']
    xy = lambda p: (left+p[0]*scale, top+p[1]*scale)
    page.box(left, top, width*scale, height*scale, 'board', '#375d50', 0, 2.5)
    page.text(50,150,f"{width:g} × {height:g} mm | Vue côté composants, non miroir",17,'muted')
    parts = {f['ref']: f for f in b['parts']}
    for f in b['parts']:
        ref = f['ref']
        if ref.startswith('TP'): continue
        if b['code']=='A' and ref=='U1':
            page.box(*xy((5, 26.5)), 54*scale, 27*scale, '#c3d3df', '#667f90', 4)
            for y in (27.3, 52.7):
                page.box(*xy((13.2, y-1.4)), 37.6*scale, 2.8*scale, '#283744', '#283744')
            page.box(*xy((59, 35)), 2*scale, 10*scale, '#e4e8ec', '#536b7c')
            page.text(*xy((32, 37)), 'DevKit ESP32', 27, 'ink', True, 'middle')
            page.text(*xy((32, 41)), 'module enfichable', 18, 'ink', False, 'middle')
            page.text(*xy((32, 45)), '30 broches / USB-C à droite', 16, 'ink', False, 'middle')
        else:
            if ref.startswith('J'): fill, edge = '#fffef3', '#758275'
            elif ref.startswith('U') or ref.startswith('Q'): fill, edge = '#30434d', '#172c36'
            elif ref == 'L1': fill, edge = '#506169', '#253e46'
            elif ref in ('C8', 'C10'): fill, edge = '#b9c7d9', '#536f8b'
            elif ref in ('D1','D2') and b['code']=='A' or ref in ('D5','D6','D7','D8') and b['code']=='B':
                fill, edge = ('#efb875', '#a56a25') if ref=='D8' else ('#70bd89', '#2c7b4f')
            elif ref.startswith('C'): fill, edge = '#d5bb8e', '#8a7658'
            elif ref.startswith('F'): fill, edge = '#f8edc9', '#9a8856'
            else: fill, edge = '#889a98', '#4c6963'
            for shape in f['fab']:
                if shape['kind']=='rect':
                    p, q = [xy(v) for v in shape['points']]
                    page.box(min(p[0],q[0]), min(p[1],q[1]), abs(p[0]-q[0]), abs(p[1]-q[1]), fill, edge)
                elif shape['kind']=='circle':
                    page.circle(*xy(shape['center']), shape['radius']*scale, fill, edge)
                elif shape.get('points'):
                    pts=[xy(v) for v in shape['points']]
                    if shape['kind']=='poly': pts.append(pts[0])
                    page.line(pts, edge, 1)
        for pad in f['pads']:
            if ref=='NT1': continue
            x,y=xy(pad['at'])
            if pad['drill']:
                page.circle(x,y, max(3.4, .72*scale), '#e1bc66', '#8c7743', .8)
                page.circle(x,y, max(1.7, pad['drill']*.43*scale), '#334947', None)
                if pad['number']=='1': page.box(x-9,y-9,18,18,None,'#1b4b42',0,1.3)
            elif not pad['circle']:
                sx,sy=pad['size']
                if abs(pad['angle'])%180==90: sx,sy=sy,sx
                page.box(x-sx*scale/2,y-sy*scale/2,sx*scale,sy*scale,'#e2bd72',None)
        if ref=='U1' and b['code']=='A': continue
        x,y=xy(f['at'])
        if ref.startswith('J'):
            page.text(x, y-14, ref, 19, 'ink', True)
        elif ref in ('C8','C10'):
            page.text(x+14,y-10,ref,18,'ink',True)
        elif ref.startswith('U') or ref in ('Q1','Q2','L1','F1','F2','NT1'):
            page.text(x,y-17,ref,17,'ink',True,'middle')
        elif ref.startswith('D'):
            page.text(x+11,y+6,ref,16,'ink',True)
    # Only these three coordinates are used as shared cable interfaces.
    for ref,col in [('J1','power'),('J6','bus'),('J7','ctrl'),('J10','ctrl')]:
        f=parts[ref]
        x,y=xy(f['at'])
        page.circle(x-14,y-1,5,col,None)

    def at(ref):
        f=parts[ref]
        if b['code']=='A' and ref=='U1': return xy((5,40))
        if ref.startswith('J'):
            rects=[s for s in f['fab'] if s['kind']=='rect']
            if rects:
                a,z=rects[0]['points']; return xy([(a[i]+z[i])/2 for i in (0,1)])
        return xy(f['at'])
    return at


def front_a(b):
    p=Page()
    p.heading('A','Interface utilisateur et cerveau commun', 'A 0.4.2 | ESP32 amovible, écran LCD, capteurs I²C et extension UART dédiée')
    at=board(p,b,400,300,8)
    p.card((400,160,270,110),'J5 · Luminosité', ['Vers le module BH1750.', 'I²C + alimentation 3,3 V.'], 'sensor',at('J5'),'top',[(535,270)])
    p.card((695,160,270,110),'J4 · Température', ['Vers TMP117 en option.', 'Capteur numérique I²C.'], 'sensor',at('J4'),'top',[(830,275)])
    p.card((990,160,290,110),'J2 · Écran LCD 20×4', ['Vers le backpack I²C.', 'Alimentation prévue : 3,3 V.'], 'bus',at('J2'),'top',[(1135,280)])
    p.card((50,360,300,155),'U1 · DevKit ESP32', ['Enfichée sur deux sockets.', 'Exécute le firmware.', 'Wi-Fi / MQTT et USB-C.', 'Module fourni séparément.'], 'gray',at('U1'),'left',[(375,437)])
    p.card((50,580,300,110),'JP1 · Cavalier VIN', ['Relie le 5 V à la DevKit.', 'Point d’isolement de banc.'], 'power',at('JP1'),'left',[(365,635),(370,635),(370,465),(976,465),(976,600)])
    p.card((50,770,300,135),'J10 · UART dédié', ['TX : GPIO17 / RX : GPIO16.', 'Vers une future carte B.', 'Indépendant des deux PWM.'], 'ctrl',at('J10'),'left',[(375,837),(385,837),(385,490),(1000,490)])
    p.card((1330,290,320,110),'J1 · Alimentation de A', ['Reçoit le 5 V régulé de B.', 'JST-XH, 2 contacts.'], 'power',at('J1'),'right',[(1305,345)])
    p.card((1330,440,320,135),'J6 · Dialogue avec B', ['I²C : GND, SDA et SCL.', 'Lit les mesures de la carte B.', 'R4 / R5 : rappels au 3,3 V.'], 'bus',at('J6'),'right',[(1305,507)])
    p.card((1330,590,320,135),'J7 · Commandes de B', ['PWM0 : demande de chauffe.', 'ENABLE : autorisation.', 'FAULT_N : retour logique.'], 'ctrl',at('J7'),'right',[(1305,657)])
    p.card((1330,755,320,110),'U2 · Régulateur 3,3 V', ['AP2112 : 5 V vers 3,3 V.', 'Alimente les périphériques.'], 'power',at('U2'),'right',[(1305,810),(1260,810),(1260,760)])
    p.card((1330,910,320,110),'D1 / D2 · Voyants', ['D1 : présence du 5 V.', 'D2 : présence du 3,3 V.'], 'diag',at('D2'),'right',[(1305,965),(1290,965),(1290,830)])
    p.card((820,990,460,110),'J3 · Encodeur et bouton', ['Rotation : navigation / réglage.', 'Appui : validation et réveil de l’écran.'], 'ctrl',at('J3'),'bottom',[(1050,955)])
    p.text(420,930,'Le LCD, les capteurs et l’encodeur sont reliés par des câbles : ils ne sont pas sur A.',18,'muted')
    p.footer('10 points de test au dos : alimentations, I²C, commandes et bouton. Repérage détaillé : TEST_POINTS.md.',
             'Mode de banc documenté : USB seul, JP1 retiré. Usage USB + VIN rapporté fonctionnel ; détail électrique à caractériser par DevKit.')
    return p


def front_b(b):
    p=Page()
    p.heading('B','Carte métier température : mesurer et chauffer', 'B 0.5.2 | Buck intégré, acquisition de la sonde, puissance et connecteur UART réservé')
    at=board(p,b,450,300,9)
    p.card((300,175,240,110),'J2 · Entrée DC', ['VIN+ et retour PGND.', 'Étude : 24 V / 100 W.'], 'power',at('J2'),'top',[(420,295)])
    p.card((570,155,300,135),'Protection d’entrée', ['F1 : fusible 6,3 A ; Q2 :', 'inversion de polarité.', 'D2 : écrêtage à qualifier.'], 'power',at('F1'),'top',[(720,295)])
    p.card((900,155,310,135),'J10 · Réserve UART', ['1 GND / 2 TX_A / 3 RX_A.', '2 et 3 non reliées sur B.', 'Pas de fonction UART ici.'], 'ctrl',at('J10'),'top',[(1055,295),(850,320)])
    p.card((1240,155,390,110),'J3 · Vers chauffage', ['Borne + et retour commuté.', 'Le courant reste sur B.'], 'power',at('J3'),'top',[(1435,275),(1300,285)])
    p.card((40,340,340,180),'U1 · Buck 5 V intégré', ['LM2596 + D9 + L1.', 'C8/C10 : filtrage ; R13 :', 'résistance série de C10.', 'F2 protège le 5 V partagé.', 'Remplace le module XL4015.'], 'power',at('U1'),'left',[(420,430)])
    p.card((40,590,340,155),'D5 à D8 · Diagnostic', ['D5 : entrée protégée.', 'D6 : 5 V ; D7 : 3,3 V.', 'D8 : commande du chauffage.', 'Ce n’est pas un wattmètre.'], 'diag',at('D5'),'left',[(420,667)])
    p.card((40,810,340,110),'NT1 · Jonction de masse', ['Relie GND et PGND.', 'Aucune isolation galvanique.'], 'gray',at('NT1'),'left',[(420,865),(430,865),(430,579)])
    p.card((1330,295,330,135),'Q1 · MOSFET chauffage', ['Interrompt le retour de charge.', 'Pilotage par impulsions PWM.', 'D4 : diode de roue libre.'], 'power',at('Q1'),'right',[(1300,362)])
    p.card((1330,440,330,135),'U3 + U5 · Pilotage', ['U3 combine PWM0 et ENABLE.', 'U5 commande la grille de Q1.', 'Grille pilotée en 5 V.'], 'ctrl',at('U5'),'right',[(1300,507),(1300,580),(1050,580)])
    p.card((1330,620,330,110),'U4 · Mesure ADS1115', ['Convertit la tension de sonde.', 'Envoie les mesures par I²C.'], 'sensor',at('U4'),'right',[(1300,675)])
    p.card((1330,770,330,110),'U2 · Régulateur 3,3 V', ['AP2112 pour l’analogique', 'et la logique de la carte B.'], 'power',at('U2'),'right',[(1300,825),(1285,825),(1285,740)])
    p.card((1330,920,330,110),'J4 · Sonde température', ['NTC passive 15 kΩ / B3920.', 'Vers le jack du panneau arrière.'], 'sensor',at('J4'),'right',[(1300,975),(1290,975),(1290,800)])
    p.card((440,990,245,110),'J1 · 5 V vers A', ['Alimente la carte A.', 'Via le fusible F2 de 1 A.'], 'power',at('J1'),'bottom',[(562,940)])
    p.card((715,990,245,110),'J6 · I²C vers A', ['GND / SDA / SCL.', 'ADS1115 : adresse 0x49.'], 'bus',at('J6'),'bottom',[(837,945)])
    p.card((990,990,290,110),'J7 · Logique depuis A', ['PWM0 / ENABLE / FAULT_N.', 'Positions 5 et 6 non reliées.'], 'ctrl',at('J7'),'bottom',[(1135,950)])
    p.text(470,925,'R5 + R6 + C4 : polarisation et filtrage de la sonde avant l’ADS1115.',18,'muted')
    p.footer('12 points de test au dos. J4 : broche 1 = +3V3_B ; broche 2 = NTC_RAW. Ce connecteur n’est pas une entrée 24 V.',
             'HEATER_LOW est un retour commuté, pas une masse. La coupure thermique indépendante reste externe. Usage à 12 V à qualifier.')
    return p


def wiring():
    p=Page()
    p.heading('↔','Comment les deux cartes travaillent ensemble', 'Alimentation et puissance séparées des signaux | Numéros = pastilles PCB, pas vue des fiches de câble')
    p.box(60,175,430,175,'white',None,14)
    p.text(88,215,'Alimentation externe DC',28,'power',True)
    p.text(88,255,'Entre sur B / J2.',23)
    p.text(88,291,'Dimensionnement étudié : 24 V / 100 W.',20)
    p.box(625,175,430,175,'white',None,14)
    p.text(653,215,'Carte B température',28,'power',True)
    p.text(653,255,'Protection + buck 5 V + commande.',22)
    p.text(653,291,'Mesure NTC par ADS1115.',22)
    p.box(1190,175,450,175,'white',None,14)
    p.text(1218,215,'Chauffage externe',28,'power',True)
    p.text(1218,255,'Relié au bornier B / J3.',22)
    p.text(1218,291,'Courant commuté par Q1 sur B.',22)
    p.arrow([(490,260),(625,260)],'power',5)
    p.arrow([(1055,260),(1190,260)],'power',5)
    p.box(625,475,430,115,'white',None,14)
    p.text(653,520,'Carte A + DevKit ESP32',27,'bus',True)
    p.text(653,556,'Interface, firmware et réseau.',22)
    p.box(60,390,430,210,'white',None,14)
    p.text(85,430,'J10 ↔ J10 · UART dédié',25,'ctrl',True)
    p.text(85,470,'1 GND / 2 TX_A / 3 RX_A.',22)
    p.text(85,506,'GPIO17 TX / GPIO16 RX sur A.',22)
    p.text(85,542,'2/3 NC sur B température.',22)
    p.text(85,576,'Câble facultatif ; logique 3,3 V.',20)
    p.arrow([(735,350),(735,475)],'power',4)
    p.text(595,420,'5 V',23,'power',True)
    p.arrow([(835,475),(835,350)],'ctrl',3)
    p.text(858,402,'PWM / ENABLE',19,'ctrl',True)
    p.arrow([(970,350),(970,475)],'bus',3)
    p.text(995,440,'Mesures I²C',19,'bus',True)
    blocks=[
        (60,650,480,185,'J1 ↔ J1 · POWER','power',[
            '1 : GND     2 : +5V_AB',
            'B fournit le 5 V régulé ; A le reçoit.',
            'JST-XH 2 contacts, câble droit.',
            'Le courant du chauffage ne passe pas ici.']),
        (610,650,480,185,'J6 ↔ J6 · I²C','bus',[
            '1 : GND     2 : SDA     3 : SCL',
            'La DevKit de A dialogue avec B.',
            'JST-XH 3 contacts, câble droit.',
            'Niveaux logiques : 3,3 V.']),
        (1160,650,480,185,'J7 ↔ J7 · Commandes','ctrl',[
            '1 GND / 2 PWM0 / 3 ENABLE / 4 FAULT_N',
            '5 PWM1 / 6 DIN0 : réservés, NC sur B.',
            'JST-XH 6 contacts, les 6 fils présents.',
            'FAULT_N n’est pas une coupure autonome.']),
        (60,880,760,200,'Périphériques raccordés à A','sensor',[
            'J2 LCD / J4 TMP117 optionnel / J5 BH1750 :',
            '1 GND / 2 +3V3_A / 3 SDA / 4 SCL.',
            'J3 encodeur : 1 A / 2 B / 3 bouton / 4 GND.',
            'Attention : la masse n’a pas la même position sur J3.']),
        (880,880,760,200,'Sonde et contrôles de B','sensor',[
            'B/J4 : 1 +3V3_B / 2 NTC_RAW. Sonde passive 15 kΩ.',
            'Jack arrière : pointe = +3V3_B ; corps = NTC_RAW.',
            'Le corps du jack n’est donc pas GND ; anneau non utilisé.',
            '22 points de test au dos : voir TEST_POINTS.md.']),
    ]
    for x,y,w,h,title,col,lines in blocks:
        p.box(x,y,w,h,'white',None,14)
        p.text(x+25,y+38,title,25,col,True)
        for i,line in enumerate(lines):p.text(x+25,y+76+i*29,line,20)
    p.footer('Les modules LCD / BH1750 / TMP117 et la DevKit sont externes aux composants assemblés sur les PCB.',
             'Guide de lecture, non plan d’assemblage usine. Rebrancher hors tension ; contrôle du brochage par continuité avant mise sous tension.')
    return p


def build():
    pdfmetrics.registerFont(TTFont('Arial','C:/Windows/Fonts/arial.ttf'))
    pdfmetrics.registerFont(TTFont('Arial-Bold','C:/Windows/Fonts/arialbd.ttf'))
    geometry_path=ROOT/'manufacturing/output/assembly_geometry.json'
    geometry=json.loads(geometry_path.read_text(encoding='utf-8'))
    for relative,expected in geometry['source_sha256'].items():
        assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==expected, 'Stale native geometry: '+relative
    boards={b['code']:b for b in geometry['boards']}
    # Meaningful guards against outdated wiring explanations.
    for code,b in boards.items():
        parts={f['ref']:f for f in b['parts']}
        for ref,nets in {'J1':['GND','+5V_AB'],'J6':['GND','I2C_SDA','I2C_SCL']}.items():
            assert [q['net'] for q in sorted(parts[ref]['pads'],key=lambda q:int(q['number']))]==nets
        assert len([f for f in b['parts'] if f['ref'].startswith('TP')]) == (10 if code=='A' else 12)
    assert {q['number']:q['net'] for q in next(f for f in boards['B']['parts'] if f['ref']=='J4')['pads']}=={'1':'+3V3_B','2':'NTC_RAW'}
    pages=[front_a(boards['A']),front_b(boards['B']),wiring()]
    OUT.mkdir(exist_ok=True,parents=True); PDF.parent.mkdir(exist_ok=True,parents=True)
    for name,page in zip(['carte_A_annotee','carte_B_annotee','liaisons_A_B'],pages):
        renderSVG.drawToFile(page.d,str(OUT/(name+'.svg')))
    paper_w,paper_h=landscape(A3)
    c=canvas.Canvas(str(PDF),pagesize=(paper_w,paper_h),invariant=1)
    c.setTitle('Cartes A et B - Guide visuel annote - A0.4.3 / B0.5.3 UART')
    c.setAuthor('Projet controleur de temperature')
    for page in pages:
        c.saveState(); c.scale(paper_w/W,paper_h/H); renderPDF.draw(page.d,c,0,0); c.restoreState(); c.showPage()
    c.save()
    subprocess.run(['pdftoppm','-scale-to','2400','-png',str(PDF),str(OUT/'guide')],check=True)
    for i,name in enumerate(['carte_A_annotee','carte_B_annotee','liaisons_A_B'],1):
        (OUT/f'guide-{i}.png').replace(OUT/(name+'.png'))
    sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),geometry_path,ROOT/'manufacturing/wiring_and_harnesses.md',ROOT/'TEST_POINTS.md',ROOT/'diagnostic_leds.json']}
    sources.update(geometry['source_sha256'])
    (OUT/'source_manifest.json').write_text(json.dumps(dict(native_revisions=['A0.4.3','B0.5.3'],
        purpose='French explanatory diagrams; not a manufacturing or placement approval',
        source_sha256=sources,pcb_sources_modified=False),indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'pdf':str(PDF),'pages':3,'native_geometry_verified':True,'output':str(OUT)},indent=2))


if __name__=='__main__': build()
