const fs = require('fs');
const occtimportjs = require('occt-import-js')();
occtimportjs.then((occt) => {
  const buf = new Uint8Array(fs.readFileSync(process.argv[2] || "robot.step"));
  const t0 = Date.now();
  const res = occt.ReadStepFile(buf, { linearUnit: 'millimeter', linearDeflectionType: 'bounding_box_ratio', linearDeflection: 0.002, angularDeflection: 0.5 });
  console.error('read', res.success, 'meshes', res.meshes.length, 'ms', Date.now() - t0);
  // compact: write meshes as binary + tree json
  const meta = { root: res.root, meshes: [] };
  const chunks = []; let off = 0;
  for (const m of res.meshes) {
    const pos = new Float32Array(m.attributes.position.array);
    const idx = new Uint32Array(m.index.array);
    meta.meshes.push({ name: m.name, color: m.color || null, nv: pos.length / 3, ni: idx.length, pOff: off, iOff: off + pos.byteLength });
    chunks.push(Buffer.from(pos.buffer)); chunks.push(Buffer.from(idx.buffer));
    off += pos.byteLength + idx.byteLength;
  }
  fs.writeFileSync('robot_meshes.bin', Buffer.concat(chunks));
  fs.writeFileSync('robot_meta.json', JSON.stringify(meta));
  console.error('bytes', off);
});
