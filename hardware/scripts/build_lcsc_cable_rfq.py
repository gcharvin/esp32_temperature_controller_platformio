"""Build/check the standalone LCSC cable quotation dossier, never submit it.

CSV tables are authored by export_lcsc_cable_tables.mjs (Artifact Tool).
Run primary Python with --documents, visually review PDF, then --package.
"""
import argparse
import csv
import hashlib
import html
import io
import json
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
FOLDER=ROOT/'hardware/manufacturing/lcsc_cable_rfq'
SPEC=FOLDER/'rfq_specification.json'
PDF=FOLDER/'output/pdf/LCSC_CABLE_RFQ_10_EACH.pdf'
ARCHIVE=FOLDER.parent/'LCSC_CABLE_RFQ_10_EACH_NOT_FOR_ORDER.zip'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def validate():
    d=read(SPEC);hs=d['harnesses'];assert len(hs)==13 and d['quantity_each']==10
    assert len({h['id'] for h in hs})==13
    assert sum(h['category'].startswith('INTERNAL') for h in hs)==8
    assert sum(h['category']=='PROVISIONAL' for h in hs)==5
    manifests=read(ROOT/'hardware/manufacturing/review_A04_B05/manifest.json')
    for relative,digest in manifests['sources_sha256'].items():
        if relative.endswith(('.kicad_sch','.kicad_pcb','.kicad_pro')):
            assert sha(ROOT/'hardware'/relative)==digest,relative
    pins={}
    for code in 'AB':
        p=ROOT/f'hardware/manufacturing/review_A04_B05/{code}/netlist.xml'
        assert sha(p)==manifests['output_sha256'][code+'/netlist.xml']
        pins[code]={(n.get('ref'),n.get('pin')):net.get('name').removeprefix('/')
                    for net in ET.parse(p).findall('./nets/net') for n in net.findall('node')}
    expected={}
    for r in csv.DictReader((ROOT/'hardware/manufacturing/harness_pin_map.csv').open(encoding='utf-8-sig')):
        ident={'POWER':'H01','I2C':'H02','CTRL':'H03'}[r['Harness']]
        h=next(x for x in hs if x['id']==ident)
        assert h['awg']==int(r['Wire_AWG']) and h['length_mm']==int(r['Length_mm'])
        assert h['tolerance_mm']==int(r['Tolerance_mm'])
        assert any(a==r['Pin_A'] and b==r['Pin_B'] for a,b,_,_ in h['map'])
        a=(r['End_A'].split('/')[1],r['Pin_A']);b=(r['End_B'].split('/')[1],r['Pin_B'])
        assert pins['A'][a]==r['Net_A']
        assert pins['B'][b]==r['Net_A'] if not r['Net_B'].startswith('NC') else pins['B'][b].startswith('unconnected-')
        expected['A/'+'.'.join(a)]=pins['A'][a];expected['B/'+'.'.join(b)]=pins['B'][b]
    periph={'J2':['GND','+3V3_A','I2C_SDA','I2C_SCL'],
            'J3':['ENC_A','ENC_B','ENC_SW','GND'],
            'J4':['GND','+3V3_A','I2C_SDA','I2C_SCL'],
            'J5':['GND','+3V3_A','I2C_SDA','I2C_SCL']}
    for ref,nets in periph.items():
        for pin,net in enumerate(nets,1):
            assert pins['A'][ref,str(pin)]==net
            expected[f'A/{ref}.{pin}']=net
    for ref,nets in {'J2':['VIN_RAW','PGND'],'J3':['VIN_PROTECTED','HEATER_LOW'],'J4':['+3V3_B','NTC_RAW']}.items():
        for pin,net in enumerate(nets,1):
            assert pins['B'][ref,str(pin)]==net
            expected[f'B/{ref}.{pin}']=net
    for h in hs:
        assert h['end_a'] in d['connectors'] and h['end_b'] in d['connectors']
        for end in ('end_a','end_b'):
            c=d['connectors'][h[end]]
            awg=h.get('tail_awg',h['awg']) if end=='end_b' else h['awg']
            if 'contact_awg_min' in c:assert c['contact_awg_min']<=awg<=c['contact_awg_max']
        if h['id']!='H05':assert len({a for a,_,_,_ in h['map']})==len(h['map'])
    assert next(h for h in hs if h['id']=='H05')['map'][-2:]==[['4','C','GND','black'],['4','SW2','GND branch','black']]
    assert next(h for h in hs if h['id']=='H08')['map']==[['1','TIP','+3V3_B','red'],['2','SLEEVE','NTC_RAW (NOT GND)','white']]
    for ident,ref in [('H04','J2'),('H06','J4'),('H07','J5')]:
        h=next(x for x in hs if x['id']==ident)
        assert [a for a,_,_,_ in h['map']]==['1','2','3','4']
        for a,_,signal,_ in h['map']:
            actual=pins['A'][ref,a]
            assert signal==actual or signal.startswith(actual+' /'),(ident,a,signal,actual)
    for a,_,signal,_ in next(h for h in hs if h['id']=='H05')['map']:
        actual=pins['A']['J3',a]
        assert signal==actual or (a=='4' and signal=='GND branch')
    for a,_,signal,_ in next(h for h in hs if h['id']=='H08')['map']:
        actual=pins['B']['J4',a]
        assert signal==actual or signal.startswith(actual+' ('),('H08',a,signal,actual)
    for ident,ref in [('H10','J2'),('H11','J3')]:
        for a,_,signal,_ in next(h for h in hs if h['id']==ident)['map']:
            assert signal.startswith(pins['B'][ref,a.split('.')[-1]]+' ')
    assert all(h['length_mm']>0 and h['tolerance_mm']>0 for h in hs)
    return d,expected


def documents():
    from reportlab.lib import colors
    from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
    from reportlab.lib.pagesizes import A4
    from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak,Flowable
    d,pins=validate();PDF.parent.mkdir(parents=True,exist_ok=True)
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='BodyRFQ',fontName='Helvetica',fontSize=9.2,leading=12,spaceAfter=7))
    styles.add(ParagraphStyle(name='SmallRFQ',fontName='Helvetica',fontSize=8,leading=10,spaceAfter=5))
    styles.add(ParagraphStyle(name='HeadRFQ',fontName='Helvetica-Bold',fontSize=16,leading=20,spaceAfter=10,textColor=colors.HexColor('#18384b')))
    def p(t,style='BodyRFQ'):return Paragraph(html.escape(str(t)),styles[style])
    def table(rows,widths):
        t=Table([[p(v,'SmallRFQ') for v in row] for row in rows],colWidths=widths,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e6edf1')),('VALIGN',(0,0),(-1,-1),'TOP'),
            ('BOTTOMPADDING',(0,0),(-1,-1),5),('TOPPADDING',(0,0),(-1,-1),5),('LINEBELOW',(0,0),(-1,0),.5,colors.HexColor('#8b9da6'))]))
        return t
    class Logical(Flowable):
        def __init__(self,h):Flowable.__init__(self);self.h=h;self.width=515;self.height=74
        def draw(self):
            c=self.canv;c.setStrokeColor(colors.HexColor('#486778'));c.setFillColor(colors.HexColor('#f2f6f8'))
            c.roundRect(0,23,130,32,4,fill=1);c.roundRect(385,23,130,32,4,fill=1)
            c.setFillColor(colors.HexColor('#18384b'));c.setFont('Helvetica-Bold',9)
            c.drawCentredString(65,35,self.h['end_a']);c.drawCentredString(450,35,self.h['end_b'])
            for offset in (-5,0,5):c.line(130,39+offset,385,39+offset)
            c.setFont('Helvetica',8);c.drawCentredString(257,61,'L = '+str(self.h['length_mm'])+' +/- '+str(self.h['tolerance_mm'])+' mm; see length definition')
            c.drawCentredString(257,8,'LOGICAL INTERCONNECT ONLY - NOT A CONNECTOR FACE VIEW')
    story=[p('LCSC Cable - quotation dossier','HeadRFQ'),p('RFQ-CABLE-01 | 09 November 2026 | 10 of EACH listed assembly'),
        p('QUOTATION ONLY - NOT RELEASED FOR PRODUCTION','HeadRFQ'),
        p('13 quotation lines /130 assemblies if all lines were selected:8 internal definitions (80pcs, including optionalTMP117) and5 provisional external/service definitions (50pcs). Price each separately. No order, sample build or supplier contact has been authorized by this package.'),
        table([['ID','Assembly','Qty','Quotation scope']]+[[h['id'],h['name'],10,h['category'].replace('_',' ')] for h in d['harnesses']],[35,260,35,185]),
        Spacer(1,8),p('Eight internal lines define connector families and electrical maps. Provisional linesH09-H13 need supplier proposals and purchaser application choices; do not treat their assumed lengths or undefined ends as manufacturing approval.'),
        p('Request separate materials/labour/tooling/sample/shipping/tax costs and lead times. List exact suppliedMPNs and every deviation. Include the encoder and panel jack as finished workshop-soldered subassemblies or price consigned parts separately.'),PageBreak()]
    story += [p('Common construction and approval requirements','HeadRFQ')]
    for label,key in [('Length datum','length_definition'),('Pin numbering and views','orientation'),('Wire, jacket and labels','internal_finish'),('Parts and substitutions','substitutions'),('Inspection','inspection')]:
        story += [p(label,'Heading3'),p(d[key])]
    story += [p('Source drawings and references','Heading3')]
    for url in d['web_sources']:story.append(p(url,'SmallRFQ'))
    story.append(PageBreak())
    for index,h in enumerate(d['harnesses']):
        a=d['connectors'][h['end_a']];b=d['connectors'][h['end_b']]
        story += [p(h['id']+' | '+h['name'],'HeadRFQ'),p('QUANTITY10 | '+h['category'].replace('_',' ')+' | QUOTATION ONLY'),
            p('End A: '+h['from']+' - '+a['housing']+'; '+a['contact']),p('End B: '+h['to']+' - '+b['housing']+'; '+b['contact']),Logical(h),
            p('Wire: '+(str(h['awg'])+'AWG' if h['awg'] else str(h.get('cross_section_mm2','TBD'))+'mm2 / supplier proposal')+(' main, '+str(h['tail_awg'])+'AWG short tails' if h.get('tail_awg') else '')+' | '+h['length_status'])]
        if h['map']:
            story += [table([['A cavity / function','B cavity / function','Signal / purpose','Colour']]+h['map'],[100,115,220,80])]
        else:
            story += [p('No discrete USB pin-to-pin table supplied: quote a complete data-capable USB extension. Supplier must document and validate the actual USB-C termination and CC implementation.')]
        if h['id'] in ('H08','H09'):story.append(p('TRS RING NOT CONNECTED. Insulate ring and unused switch contacts. SLEEVE = NTC_RAW, NOT GND. Identify physical solder lugs by continuity with a known plug.'))
        if h['id']=='H05':story.append(p('Only intentional branch: A/J3.4 GND -> encoderC AND switchSW2. No connection from GND to A/B/SW1 except through normal encoder/switch contacts. Test unpowered state and switch operation, not an indiscriminate insulation test across its contacts.'))
        story += [p('Jacket and finish','Heading3'),p(h['finish']),p('Approval points / supplier response','Heading3'),p(h['approval']),
            p('End reference notes','Heading3'),p(a.get('note','Use exact manufacturer cavity numbering and approved crimp/wire combination.'),'SmallRFQ'),p(b.get('note','Use exact manufacturer cavity numbering and approved crimp/wire combination.'),'SmallRFQ'),
            p('Manufacturer references: '+a['source']+' | '+b['source'],'SmallRFQ')]
        if index<len(d['harnesses'])-1:story.append(PageBreak())
    def footer(c,doc):
        c.setStrokeColor(colors.HexColor('#b6c3cc'));c.line(40,32,555,32);c.setFont('Helvetica',8)
        c.drawString(40,20,'RFQ-CABLE-01 | 2026-11-09 | QUOTATION ONLY | 10 EACH');c.drawRightString(555,20,str(doc.page))
    SimpleDocTemplate(str(PDF),pagesize=A4,rightMargin=40,leftMargin=40,topMargin=35,bottomMargin=44,
        title='LCSC Cable RFQ -10 of each harness -not for production',author='Laboratory hardware prototype').build(story,onFirstPage=footer,onLaterPages=footer)
    (FOLDER/'pcb_pin_extract.json').write_text(json.dumps(dict(status='Maps checked against source-bound KiCad netlists',pins=pins),indent=2)+'\n',encoding='utf-8')
    print('Generated '+str(PDF))


def package():
    from pypdf import PdfReader
    d,pins=validate();pages=PdfReader(PDF).pages
    assert len(pages)==15,('PDF overflow or missing harness page',len(pages))
    assert all(('H%02d'%i) in pages[i+1].extract_text() for i in range(1,14))
    def rows(name):return list(csv.DictReader((FOLDER/name).open(encoding='utf-8-sig',newline='')))
    lines=rows('quote_lines.csv');assert len(lines)==13 and all(int(r['Quantity'])==10 for r in lines)
    assert {r['ID'] for r in lines}=={h['id'] for h in d['harnesses']}
    mapping=rows('pin_to_pin.csv');assert len(mapping)==38
    wanted=[(h['id'],str(a),str(b),s,c) for h in d['harnesses'] for a,b,s,c in h['map']]
    assert [(r['ID'],r['End_A_pin_or_function'],r['End_B_pin_or_function'],r['Signal'],r['Colour']) for r in mapping]==wanted
    assert len(rows('connector_requirements.csv'))==len(d['connectors'])
    qa=read(FOLDER/'visual_review.json');assert qa['status']=='PASS' and qa['pdf_sha256']==sha(PDF)
    for name,digest in qa['tables_sha256'].items():assert sha(FOLDER/name)==digest,'Stale table review: '+name
    source_files=[ROOT/r for r in d['source_documents']]+[SPEC,Path(__file__),ROOT/'hardware/scripts/export_lcsc_cable_tables.mjs']
    result=dict(status='PASS_LOCAL_RFQ_DOCUMENTS_ONLY',revision=d['revision'],quotation_lines=13,quantity_each=10,total_if_all_selected=130,
        internal_lines=8,provisional_lines=5,pin_mapping_rows=38,pdf_pages=len(pages),native_pcb_pin_entries_checked=len(pins),
        manufacturer_orientation_drawing_approved=False,physical_cable_fit_qualified=False,
        submitted=False,order_placed=False,production_release=False,
        source_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in source_files})
    (FOLDER/'validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    names=['README.md','request_quote_EN.txt','rfq_specification.json','quote_lines.csv','pin_to_pin.csv',
           'connector_requirements.csv','pcb_pin_extract.json','validation.json','visual_review.json',PDF.relative_to(FOLDER).as_posix()]
    files={n:(FOLDER/n).read_bytes() for n in names}
    # Existing A/B source table is attached unchanged as independent reference.
    files['reference/AB_existing_pin_contract.csv']=(ROOT/'hardware/manufacturing/harness_pin_map.csv').read_bytes()
    manifest=dict(status=d['status'],files_sha256={n:hashlib.sha256(v).hexdigest() for n,v in files.items()},
                  source_sha256=result['source_sha256'],submitted=False,order_placed=False)
    (FOLDER/'package_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    files['package_manifest.json']=(FOLDER/'package_manifest.json').read_bytes()
    with zipfile.ZipFile(ARCHIVE,'w',zipfile.ZIP_DEFLATED) as z:
        for n,v in sorted(files.items()):z.writestr(n,v)
    with zipfile.ZipFile(ARCHIVE) as z:
        assert z.testzip() is None and len(z.namelist())==len(set(z.namelist()))==len(files)
        for n,v in files.items():assert z.read(n)==v
    ARCHIVE.with_suffix('.zip.sha256').write_text(sha(ARCHIVE)+'  '+ARCHIVE.name+'\n',encoding='ascii')
    print(json.dumps(dict(status=result['status'],archive=str(ARCHIVE),members=len(files),pdf_pages=len(pages),quantity_each=10,submitted=False),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--documents',action='store_true');parser.add_argument('--package',action='store_true');args=parser.parse_args()
    if args.documents:documents()
    if args.package:package()
    if not(args.documents or args.package):validate();print('PASS: cable specification matches source-bound PCB pin maps.')
