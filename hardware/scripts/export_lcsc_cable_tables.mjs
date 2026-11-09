// Three raw CSV tables for the standalone supplier RFQ, authored with Artifact Tool.
// ARTIFACT_NODE_MODULES may point to the shared runtime's node_modules directory.
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import {createRequire} from 'node:module';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';
const modules=process.env.ARTIFACT_NODE_MODULES || path.join(os.homedir(),'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules');
const require=createRequire(path.join(modules,'__rfq_loader.cjs'));
const {Workbook}=await import(pathToFileURL(require.resolve('@oai/artifact-tool')).href);
const root=process.argv[2]||process.cwd();
const folder=path.join(root,'hardware/manufacturing/lcsc_cable_rfq');
const d=JSON.parse(await fs.readFile(path.join(folder,'rfq_specification.json'),'utf8'));
const qty=d.quantity_each;assert.equal(qty,10);assert.equal(d.harnesses.length,13);
const quoteHeaders=['ID','Assembly','Quantity','Scope','End_A','End_B','Wire','Length_mm','Tolerance_mm','Jacket_and_finish','Approval_holds','Unit_price_supplier_fill','Setup_cost_supplier_fill','Lead_days_supplier_fill','Exact_MPNs_supplier_fill','Deviations_supplier_fill'];
const quoteRows=d.harnesses.map(h=>[h.id,h.name,qty,h.category,d.connectors[h.end_a].housing,d.connectors[h.end_b].housing,
 h.awg?`${h.awg}AWG${h.tail_awg?' main / '+h.tail_awg+'AWG SH tails':''}`:(h.cross_section_mm2?h.cross_section_mm2+'mm2':'Supplier proposal'),h.length_mm,h.tolerance_mm,h.finish,h.approval,'','','','','']);
const pinHeaders=['ID','End_A','End_A_pin_or_function','End_B','End_B_pin_or_function','Signal','Colour','Notes'];
const pinRows=d.harnesses.flatMap(h=>h.map.map(([a,b,s,c])=>[h.id,h.from,a,h.to,b,s,c,
 h.id==='H05'&&a==='4'?'Intentional GND branch C+SW2':h.id==='H08'||h.id==='H09'?'RING isolated; sleeve is NTC_RAW, NOT GND':h.id==='H03'&&(a==='5'||a==='6')?'Wire present; B temperature PCB pad is NC':'Electrical numbers/functions, not drawing left-to-right indices']));
const componentHeaders=['Connector_ID','Housing_or_completed_end','Contact_or_termination','Pitch_mm','Cavities_or_functions','Internal_8_lines_end_sets','Housing_units_internal_8_lines','Crimp_contacts_internal_8_lines','Wire_range_AWG','Insulation_OD_mm','Manufacturer_source','Notes'];
const componentRows=Object.entries(d.connectors).map(([key,c])=>{
 const n=d.harnesses.filter(h=>h.category.startsWith('INTERNAL')).reduce((sum,h)=>sum+(h.end_a===key?qty:0)+(h.end_b===key?qty:0),0);
 const socket=key==='FLY4';const crimp=key.startsWith('XH')||key==='SH4'||socket;
 return[key,c.housing,c.contact,c.pitch_mm??'',c.cavities,n,n*(socket?4:1),crimp?n*c.cavities:0,
 c.contact_awg_min?`${c.contact_awg_min}-${c.contact_awg_max}`:'Supplier proposes',c.insulation_od_mm?.join('..')??'Per selected contact drawing',c.source,c.note??'Exact manufacturer combination; no substitutions by appearance'];
});
const wb=Workbook.create();
const tables=[['Quote',quoteHeaders,quoteRows,'quote_lines.csv','A1:F6'],['Pin map',pinHeaders,pinRows,'pin_to_pin.csv','A1:H8'],['Connectors',componentHeaders,componentRows,'connector_requirements.csv','A1:F6']];
const preview=path.join(os.tmpdir(),'lcsc-cable-rfq-previews');await fs.mkdir(preview,{recursive:true});
for(const [name,headers,rows]of tables){
 const sh=wb.worksheets.add(name);sh.showGridLines=false;
 sh.getRangeByIndexes(0,0,rows.length+1,headers.length).values=[headers,...rows];
 const range=sh.getUsedRange();range.format.columnWidth=24;range.format.rowHeight=72;range.format.wrapText=true;range.format.verticalAlignment='center';range.format.font.size=11;
 const top=sh.getRangeByIndexes(0,0,1,headers.length);top.format.fill='#203c50';top.format.font.color='#ffffff';top.format.font.bold=true;top.format.horizontalAlignment='center';top.format.rowHeight=42;
 sh.getRangeByIndexes(0,0,rows.length+1,1).format.columnWidth=11;
 sh.freezePanes.freezeRows(1);
}
wb.recalculate();
assert.equal(quoteRows.reduce((n,r)=>n+r[2],0),130);assert.equal(pinRows.length,38);
console.log((await wb.inspect({kind:'table',range:'Quote!A1:D14',tableMaxRows:14,tableMaxCols:4,maxChars:2300})).ndjson);
for(const [name,headers,rows,file,view]of tables){
 const sh=wb.worksheets.getItem(name);
 const quote=v=>{const s=String(v??'');return /[",\r\n]/.test(s)?'"'+s.replaceAll('"','""')+'"':s;};
 // Artifact Tool has no documented CSV-file export in the supplied API.
 // Serialize the authored worksheet's values without changing their content.
 await fs.writeFile(path.join(folder,file),sh.getUsedRange().values.map(r=>r.map(quote).join(',')).join('\r\n')+'\r\n','utf8');
 const rendered=await wb.render({sheetName:name,range:view,scale:1.4,format:'png'});
 await fs.writeFile(path.join(preview,name.replaceAll(' ','_')+'.png'),new Uint8Array(await rendered.arrayBuffer()));
}
console.log('Exported3 CSV tables;13lines,130quoted assemblies,38pin-map rows. Previews: '+preview);
