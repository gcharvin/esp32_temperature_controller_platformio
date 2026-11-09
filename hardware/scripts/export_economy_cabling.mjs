// Author the mixed catalogue/custom cable plan and update the customer CSV.
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import {createRequire} from 'node:module';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';
const modules=process.env.ARTIFACT_NODE_MODULES||path.join(os.homedir(),'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules');
const {Workbook}=await import(pathToFileURL(createRequire(path.join(modules,'__loader.cjs')).resolve('@oai/artifact-tool')).href);
const root=process.argv[2]||process.cwd();const folder=path.join(root,'hardware/manufacturing/economy_cabling');
const d=JSON.parse(await fs.readFile(path.join(folder,'economy_plan.json'),'utf8'));
const tables=[
 ['Custom',['ID','Description','Qty_base','Qty_total_with_TMP','End_A','End_B','AWG','Length_mm','Tolerance_mm','Finish','Unit_price_supplier','Setup_supplier','Lead_time_supplier'],d.custom.map(h=>[h.id,h.name,h.quantity,h.quantity+h.optional_extra_quantity,d.connectors[h.end_a].housing,d.connectors[h.end_b].housing,h.awg,h.length_mm,h.tolerance_mm,h.finish,'','','']),'custom_quote_lines.csv'],
 ['Pins',['ID','A_pin','B_pin','Function','Colour_note'],d.custom.flatMap(h=>h.map.map(r=>[h.id,...r])),'custom_pin_map.csv'],
 ['Allocation',['Old_ID','Function','Quantity_for_10_sets','Route','Implementation'],d.allocations.map(r=>[r.id,r.name,r.quantity_for_10_sets,r.route,r.implementation]),'purchase_plan.csv'],
 ['Retail',['Item','Supplier','Reference','Qty_base','Qty_optional','Unit','Public_price_TTC_EUR','URL','Status','Work_remaining'],d.retail.map(r=>[r.item,r.supplier,r.reference,r.qty_base,r.qty_optional,r.unit,r.unit_price_TTC_EUR??'',r.url,r.status,r.work]),'retail_basket.csv']
];
const wb=Workbook.create();
for(const [name,headers,rows]of tables){const sh=wb.worksheets.add(name);sh.showGridLines=false;sh.getRangeByIndexes(0,0,rows.length+1,headers.length).values=[headers,...rows];
 const r=sh.getUsedRange();r.format.columnWidth=26;r.format.rowHeight=85;r.format.wrapText=true;r.format.font.size=11;r.format.verticalAlignment='center';
 const top=sh.getRangeByIndexes(0,0,1,headers.length);top.format.fill='#203c50';top.format.font.color='#ffffff';top.format.font.bold=true;top.format.rowHeight=40;
 sh.getRangeByIndexes(0,0,rows.length+1,1).format.columnWidth=name==='Retail'?25:14;
}
const bomPath=path.join(root,'hardware/manufacturing/system_bom_review.csv');
const customer=await Workbook.fromCSV(await fs.readFile(bomPath,'utf8'),{sheetName:'Customer'});
const sh=customer.worksheets.getItem('Customer');const v=sh.getUsedRange().values;const head=v[0];assert.equal(v.length,26);assert.equal(head.length,16);
const originalQty=new Map(v.slice(1).map(r=>[r[0],r[1]]));
const updates={
 ENCODER:{Purchase_action_FR:'GoTronicADA377 ; souder le cordon universelU04 en atelier',Purchase_notes_FR:'Utilisateur accepte quelques soudures, aucun sertissage. Couper les4femelles duU04 côté encodeur, dénuder et souder A/B/SW/GND ; pont communC-SW2, soudures isolées. RéférenceGoTronic38383 5,90EURTTC observée09/11. Main-d’œuvre à chiffrer, pas gratuite par hypothèse.',Harness_completion_FR:'PrésertiU04 puis5soudures de terminaux encodeur, dont commun ponté ; aucune pince'},
 TMP117:{Purchase_action_FR:'Option séparée : moduleADA4821 si souhaité ; cordonGoTronicQW-4C +U04',Purchase_notes_FR:'Pas de câbleXH-SH spécial ni de transition de calibres à commander. U04 300mm femelles individuelles +QW-4C49152 150mm mâles/SH fini à2,10EURTTC ; jonction à immobiliser et ordreGND/VIN/SDA/SCL à contrôler. Environ450mm cumulés, comportementI2C/mou à vérifier. Option10U04 supplémentaires, pas incluse dans panier de base.'},
 AB_POWER_HARNESS:{Preferred_supplier_FR:'LCSC Cable — sur mesure limité',Product_or_supplier_URL:'https://lcsccable.com/customize',Procurement_status_FR:'Devis réduitH01 ; aucun article catalogue exact qualifié',Purchase_action_FR:'Faire chiffrer10H01 standards, sans gaine spéciale',Purchase_notes_FR:'Contrat conservé : XHP2femelle-femelle,300±10mm,AWG22,droit1->1/2->2. Fils individuellement isolés, finition standard, numéros de cavités. Sur-mesure seulement faute de référenceAmazon/GoTronic exacte ; aucun kit15cm substitué.',Harness_completion_FR:'Livré serti/monté par câbleur ; aucun sertissage utilisateur'},
 AB_I2C_HARNESS:{Preferred_supplier_FR:'LCSC Cable — sur mesure limité',Product_or_supplier_URL:'https://lcsccable.com/customize',Procurement_status_FR:'Devis réduitH02 ; aucun article catalogue exact qualifié',Purchase_action_FR:'Faire chiffrer10H02 standards, sans gaine spéciale',Purchase_notes_FR:'DeuxXHP3femelles,300±10mm,AWG26,droit1->1 à3->3. Pas de raccourcissement15cm ou remplacement parAWG22 sans revue. Repérage de cavités et fils isolés standards.',Harness_completion_FR:'Livré serti/monté par câbleur ; aucune pince'},
 AB_CTRL_HARNESS:{Preferred_supplier_FR:'LCSC Cable — sur mesure limité',Product_or_supplier_URL:'https://lcsccable.com/customize',Procurement_status_FR:'Devis réduitH03 ; aucun article catalogue exact qualifié',Purchase_action_FR:'Faire chiffrer10H03 standards, six conducteurs',Purchase_notes_FR:'DeuxXHP6femelles,300±10mm,AWG26,droit1->1 à6->6. Lespositions5/6 sont câblées maisNC surB température. Pas de gaine spéciale imposée, pas deSH6 ou kit15cm substitué.',Harness_completion_FR:'Livré serti/monté par câbleur ; aucune pince'},
 NTC_INTERNAL_HARNESS:{Purchase_action_FR:'GoTronicJST1164 +tresseGAIT06 ; deux soudures au jack',Purchase_notes_FR:'Cordon49197 préserti15cm à2,10EURTTC ; vérifier longueur réelle et mou de dépose avant achat du lot, section non publiée. Protection interneGAIT06/48620 polyester3..9mm à0,80EUR/m (2m pour10cordons15cm +marge), pas une isolation supplémentaire ni une gaine de sonde extérieure. Isoler séparément les2soudures, retenir la tresse. J4.1->tip ; J4.2->sleeveNTC_RAW, pasGND ; ringNC. Si15cm insuffisant, reconsulter pour30cm avant achat.',Harness_completion_FR:'Préserti catalogue ;2soudures au jack ; aucune pince ;15cm conditionnels'},
 PERIPHERAL_HARNESSES:{Preferred_supplier_FR:'LCSC Cable U04 commun + GoTronic',Reference_or_candidate:'U04 universal XHP4 to4female1-way sockets; optionalGoTronicQW-4C',Product_or_supplier_URL:'https://lcsccable.com/customize',Procurement_status_FR:'Une référence universelle à chiffrer ;30base +10TMP optionnels',Purchase_action_FR:'U04 communLCD/BH1750/encodeur ; QW-4C catalogue pourTMP optionnel',Purchase_notes_FR:'Pour10appareils :30U04 en base (10LCD,10BH1750,10encodeur), ou40U04 au total siTMP117 monté. U04 :XHP4->4femelles2,54mm individuelles,AWG26,300±10mm,numéros1..4 aux2bouts, aucune fonctionGND/VCC universelle. Encodeur : couper lesfemelles et souder en atelier, pin4GND et paspin1 ; LCD/BH/TMP :pin1GND,2=3V3,3=SDA,4=SCL. TMP ajouteQW-4C49152fini, pas de sur-mesureSH. Pas de cordonXH4catalogue exact qualifié chezAmazon/GoTronic ; pas de faux achat prêt à brancher.',Harness_completion_FR:'U04 déjà serti ;LCD/BH enfichables, encodeur à souder ;TMP optionnel avecQW-4C'},
 POWER_WIRING:{Purchase_action_FR:'Câble catalogue gainé0,75mm2 ; dénuder/serrer côtéPhoenix sans embout obligatoire',Purchase_notes_FR:'Phoenix1715721 accepte fil souple0,14..1,5mm2 directement :0,75mm2 conforme, un conducteur parpôle,7mm dénudés,0,5..0,6Nm en soutenant le bornier. Aucun sertissage requis côtéPCB si montage propre sans brin libre ; ne pas étamer. Embouts prémontés restent une option. Gaine commune adaptée3..6,5mm auxpresse-étoupes, isolation et terminaison extérieure à choisir avecchauffage/alimentation ; aucun câble exact gelé.',Harness_completion_FR:'CôtéPCB :dénudage et serrage sans sertissage ;autres extrémités à qualifier',Procurement_status_FR:'Catalogue visé ;référence/longueur/appareils encore ouverts'}
};
for(let i=1;i<v.length;i++){for(const [key,value]of Object.entries(updates[v[i][0]]??{}))v[i][head.indexOf(key)]=value;assert.equal(v[i][1],originalQty.get(v[i][0]));}
sh.getRangeByIndexes(0,0,v.length,head.length).values=v;
wb.recalculate();customer.recalculate();
assert.equal(d.custom.reduce((s,h)=>s+h.quantity,0),60);assert.equal(d.custom.reduce((s,h)=>s+h.quantity+h.optional_extra_quantity,0),70);
const quote=x=>{const s=String(x??'');return /[",\r\n]/.test(s)?'"'+s.replaceAll('"','""')+'"':s;};
const save=async(p,values)=>fs.writeFile(p,values.map(r=>r.map(quote).join(',')).join('\r\n')+'\r\n','utf8');
const previews=path.join(os.tmpdir(),'economy-cabling-previews');await fs.mkdir(previews,{recursive:true});
for(const [name,headers,rows,file]of tables){const sheet=wb.worksheets.getItem(name);await save(path.join(folder,file),sheet.getUsedRange().values);
 const range=name==='Custom'?'A1:F5':name==='Pins'?'A1:E8':name==='Retail'?'A1:G5':'A1:E6';
 const im=await wb.render({sheetName:name,range,scale:1,format:'png'});await fs.writeFile(path.join(previews,name+'.png'),new Uint8Array(await im.arrayBuffer()));}
await save(bomPath,sh.getUsedRange().values);
console.log((await wb.inspect({kind:'table',range:'Custom!A1:D5',tableMaxRows:5,tableMaxCols:4,maxChars:1000})).ndjson);
console.log('Saved4 economy CSVs +customer BOM;25customer rows/quantities preserved;60base or70withTMPcustomcables.');
