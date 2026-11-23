// 실기핏 서버: 소속(그룹)별 기록 저장과 판정 기준 배포. Vercel 함수 + Neon Postgres.
const { neon } = require('@neondatabase/serverless');
const crypto = require('crypto');
const url = process.env.DATABASE_URL || process.env.POSTGRES_URL;
const sql = url ? neon(url) : null;
let ready;
const init = () => ready || (ready = (async () => {
  await sql`CREATE TABLE IF NOT EXISTS groups (code text PRIMARY KEY, key_hash text NOT NULL, join_hash text, cfg jsonb, cfg_ver int NOT NULL DEFAULT 0, created_at timestamptz NOT NULL DEFAULT now())`;
  await sql`CREATE TABLE IF NOT EXISTS records (id bigserial PRIMARY KEY, grp text NOT NULL, name text NOT NULL, ev text NOT NULL, value real NOT NULL, unit text NOT NULL, ok boolean NOT NULL, note text, info boolean, at timestamptz NOT NULL DEFAULT now())`;
  await sql`CREATE INDEX IF NOT EXISTS records_grp_at ON records (grp, at DESC)`;
  try { await sql`ALTER TABLE groups ADD COLUMN IF NOT EXISTS join_hash text`; } catch (e) { }
  await sql`CREATE TABLE IF NOT EXISTS uni_ranks (id bigserial PRIMARY KEY, uni text NOT NULL, adm text NOT NULL, uid text NOT NULL, name text NOT NULL, sex text NOT NULL, score real NOT NULL, mx real NOT NULL, recs jsonb, at timestamptz NOT NULL DEFAULT now())`;
  await sql`CREATE INDEX IF NOT EXISTS uni_ranks_uni ON uni_ranks (uni, adm)`;
  await sql`CREATE TABLE IF NOT EXISTS shares (id bigserial PRIMARY KEY, grp text NOT NULL, name text NOT NULL, ev text NOT NULL, value real NOT NULL, unit text NOT NULL, ok boolean NOT NULL, note text, data jsonb, video_url text, at timestamptz NOT NULL DEFAULT now())`;
})().catch((e) => { ready = null; throw e; }));
const hash = (s) => crypto.createHash('sha256').update('silgi:' + s).digest('hex');
const str = (v, n) => String(v ?? '').trim().slice(0, n);
const EVS = ['jump', 'situp', 'run10', 'run20', 'shuttle', 'flex', 'sargent'];
const row = (r) => ({ id: Number(r.id), name: r.name, ev: r.ev, value: r.value, unit: r.unit, ok: r.ok, note: r.note || '', info: !!r.info, at: r.at });

module.exports = async (req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  const fail = (code, error) => res.status(code).json({ error });
  if (!sql) return fail(503, '서버 DB가 아직 연결되지 않았습니다.');
  try {
    await init();
    const q = req.query || {}, b = req.body && typeof req.body === 'object' ? req.body : {}, a = str(q.a, 20);
    // ── 대학별 랭킹: 소속과 상관없이 같은 대학·모집 시기에 기록을 올린 사람끼리 본다
    if (a === 'uniAdd' || a === 'uniRank' || a === 'uniDel') {
      const uni = str(b.uni ?? q.uni, 60), adm = str(b.adm ?? q.adm, 4), uid = str(b.uid ?? q.uid, 40);
      if (!uni || !['정시', '수시'].includes(adm)) return fail(400, '대학 정보가 맞지 않습니다.');
      if (a !== 'uniRank' && uid.length < 8) return fail(400, '기기 정보가 없습니다.');
      if (a === 'uniAdd') {
        const nm = str(b.name, 30), sex = str(b.sex, 1), score = Number(b.score), mx = Number(b.mx), recs = JSON.stringify(b.recs || {});
        if (!nm || !['m', 'f'].includes(sex) || !isFinite(score) || !isFinite(mx) || recs.length > 2000) return fail(400, '이름·성별·기록을 확인해 주세요.');
        await sql`DELETE FROM uni_ranks WHERE uni = ${uni} AND adm = ${adm} AND uid = ${uid}`;
        await sql`INSERT INTO uni_ranks (uni, adm, uid, name, sex, score, mx, recs) VALUES (${uni}, ${adm}, ${uid}, ${nm}, ${sex}, ${score}, ${mx}, ${recs}::jsonb)`;
      }
      if (a === 'uniDel') await sql`DELETE FROM uni_ranks WHERE uni = ${uni} AND adm = ${adm} AND uid = ${uid}`;
      const rows = await sql`SELECT uid, name, sex, score, mx, recs, at FROM uni_ranks WHERE uni = ${uni} AND adm = ${adm} ORDER BY score DESC, at ASC LIMIT 300`;
      return res.json({ rows: rows.map(r => ({ me: !!uid && r.uid === uid, name: r.name, sex: r.sex, score: r.score, mx: r.mx, recs: typeof r.recs === 'string' ? JSON.parse(r.recs) : r.recs, at: r.at })) });
    }
    const grp = str(b.grp ?? q.grp, 30).toLowerCase(), key = str(b.key ?? q.key, 100), name = str(b.name ?? q.name, 30);
    if (!/^[a-z0-9가-힣_-]{2,30}$/.test(grp)) return fail(400, '소속 코드는 2~30자의 한글·영문·숫자로 입력해 주세요.');
    const g = (await sql`SELECT * FROM groups WHERE code = ${grp}`)[0], pw = str((Buffer.isBuffer(b) ? null : b.pw) ?? q.pw, 100);
    if (a === 'create') {
      if (g) return fail(409, '이미 쓰이고 있는 소속 코드입니다. 다른 코드를 정해 주세요.');
      if (key.length < 4) return fail(400, '관리자 비밀번호는 4자 이상으로 정해 주세요.');
      await sql`INSERT INTO groups (code, key_hash, join_hash) VALUES (${grp}, ${hash(key)}, ${pw ? hash(pw) : null})`;
      return res.json({ ok: true });
    }
    if (!g) return fail(404, '없는 소속 코드입니다. 코치에게 코드를 확인해 주세요.');
    const admin = !!key && g.key_hash === hash(key);
    if (key && !admin) return fail(403, '관리자 비밀번호가 맞지 않습니다.');
    if (g.join_hash && !admin && hash(pw) !== g.join_hash) return fail(403, '참여 비밀번호가 맞지 않습니다.');
    if (a === 'setpw') { if (!admin) return fail(403, '관리자만 바꿀 수 있습니다.'); await sql`UPDATE groups SET join_hash = ${pw ? hash(pw) : null} WHERE code = ${grp}`; return res.json({ ok: true }); }
    const cfgOut = () => (String(q.ver ?? b.ver ?? '') === String(g.cfg_ver) ? {} : { cfg: g.cfg });
    if (a === 'join') return res.json({ ok: true, admin, locked: !!g.join_hash, ver: g.cfg_ver, cfg: g.cfg });
    if (a === 'add') {
      const value = Number(b.value);
      if (!name || !EVS.includes(b.ev) || !isFinite(value)) return fail(400, '기록 형식이 맞지 않습니다.');
      const r = await sql`INSERT INTO records (grp, name, ev, value, unit, ok, note, info) VALUES (${grp}, ${name}, ${b.ev}, ${value}, ${str(b.unit, 8)}, ${!!b.ok}, ${str(b.note, 200)}, ${!!b.info}) RETURNING id, at`;
      return res.json({ id: Number(r[0].id), at: r[0].at });
    }
    if (a === 'list') {
      if (!admin && !name) return fail(400, '이름이 필요합니다.');
      const rows = name ? await sql`SELECT * FROM records WHERE grp = ${grp} AND name = ${name} ORDER BY at DESC LIMIT 500`
        : await sql`SELECT * FROM records WHERE grp = ${grp} ORDER BY at DESC LIMIT 3000`;
      return res.json(Object.assign({ rows: rows.map(row), ver: g.cfg_ver }, cfgOut()));
    }
    if (a === 'del') {
      const id = Number(b.id); if (!isFinite(id)) return fail(400, '기록 번호가 없습니다.');
      if (admin) await sql`DELETE FROM records WHERE id = ${id} AND grp = ${grp}`;
      else if (name) await sql`DELETE FROM records WHERE id = ${id} AND grp = ${grp} AND name = ${name}`;
      else return fail(400, '이름이 필요합니다.');
      return res.json({ ok: true });
    }
    if (a === 'cfg') {
      if (!admin) return fail(403, '관리자만 기준을 바꿀 수 있습니다.');
      const r = await sql`UPDATE groups SET cfg = ${JSON.stringify(b.cfg || {})}::jsonb, cfg_ver = cfg_ver + 1 WHERE code = ${grp} RETURNING cfg_ver`;
      return res.json({ ok: true, ver: r[0].cfg_ver });
    }
    // ── 소속 공유(관절선 재생 + 선택적으로 영상)
    const srow = (r) => ({ id: Number(r.id), name: r.name, ev: r.ev, value: r.value, unit: r.unit, ok: r.ok, note: r.note || '', video: !!r.video_url, at: r.at });
    if (a === 'share') {
      const value = Number(b.value), data = JSON.stringify(b.data || {});
      if (!name || !EVS.includes(b.ev) || !isFinite(value)) return fail(400, '공유 형식이 맞지 않습니다.');
      if (data.length > 2000000) return fail(413, '공유할 자료가 너무 큽니다. 분석 구간을 줄여 주세요.');
      const r = await sql`INSERT INTO shares (grp, name, ev, value, unit, ok, note, data) VALUES (${grp}, ${name}, ${b.ev}, ${value}, ${str(b.unit, 8)}, ${!!b.ok}, ${str(b.note, 200)}, ${data}::jsonb) RETURNING id`;
      return res.json({ id: Number(r[0].id) });
    }
    if (a === 'feed') return res.json({ rows: (await sql`SELECT id, name, ev, value, unit, ok, note, video_url, at FROM shares WHERE grp = ${grp} ORDER BY at DESC LIMIT 300`).map(srow) });
    const sid = Number((Buffer.isBuffer(b) ? null : b.id) ?? q.id), sh = isFinite(sid) && (a === 'shareGet' || a === 'shareDel' || a === 'video') ? (await sql`SELECT * FROM shares WHERE id = ${sid} AND grp = ${grp}`)[0] : null;
    if (a === 'shareGet') { if (!sh) return fail(404, '지워졌거나 없는 공유입니다.'); return res.json(Object.assign(srow(sh), { data: sh.data, video_url: sh.video_url })); }
    if (a === 'shareDel') {
      if (!sh) return res.json({ ok: true });
      if (!admin && sh.name !== name) return fail(403, '올린 사람과 관리자만 지울 수 있습니다.');
      if (sh.video_url) { try { await require('@vercel/blob').del(sh.video_url); } catch (e) { } }
      await sql`DELETE FROM shares WHERE id = ${sid} AND grp = ${grp}`; return res.json({ ok: true });
    }
    if (a === 'video') {
      if (!process.env.BLOB_READ_WRITE_TOKEN) return fail(503, '영상 저장소가 아직 연결되지 않았습니다.');
      if (!sh || sh.name !== name) return fail(403, '내가 올린 공유에만 영상을 붙일 수 있습니다.');
      if (!Buffer.isBuffer(b) || !b.length) return fail(400, '영상 파일이 비어 있습니다.');
      const type = str(req.headers['x-vtype'], 60) || 'video/webm', out = await require('@vercel/blob').put(`silgi/${hash(grp).slice(0, 12)}/${sid}.${/mp4/.test(type) ? 'mp4' : 'webm'}`, b, { access: 'public', contentType: type, addRandomSuffix: true });
      await sql`UPDATE shares SET video_url = ${out.url} WHERE id = ${sid} AND grp = ${grp}`; return res.json({ ok: true, url: out.url });
    }
    return fail(400, '알 수 없는 요청입니다.');
  } catch (e) { return fail(500, '서버 오류: ' + e.message); }
};
