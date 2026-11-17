// 실기핏 서버: 소속(그룹)별 기록 저장과 판정 기준 배포. Vercel 함수 + Neon Postgres.
const { neon } = require('@neondatabase/serverless');
const crypto = require('crypto');
const url = process.env.DATABASE_URL || process.env.POSTGRES_URL;
const sql = url ? neon(url) : null;
let ready;
const init = () => ready || (ready = (async () => {
  await sql`CREATE TABLE IF NOT EXISTS groups (code text PRIMARY KEY, key_hash text NOT NULL, cfg jsonb, cfg_ver int NOT NULL DEFAULT 0, created_at timestamptz NOT NULL DEFAULT now())`;
  await sql`CREATE TABLE IF NOT EXISTS records (id bigserial PRIMARY KEY, grp text NOT NULL, name text NOT NULL, ev text NOT NULL, value real NOT NULL, unit text NOT NULL, ok boolean NOT NULL, note text, info boolean, at timestamptz NOT NULL DEFAULT now())`;
  await sql`CREATE INDEX IF NOT EXISTS records_grp_at ON records (grp, at DESC)`;
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
    const grp = str(b.grp ?? q.grp, 30).toLowerCase(), key = str(b.key ?? q.key, 100), name = str(b.name ?? q.name, 30);
    if (!/^[a-z0-9가-힣_-]{2,30}$/.test(grp)) return fail(400, '소속 코드는 2~30자의 한글·영문·숫자로 입력해 주세요.');
    const g = (await sql`SELECT code, key_hash, cfg, cfg_ver FROM groups WHERE code = ${grp}`)[0];
    if (a === 'create') {
      if (g) return fail(409, '이미 쓰이고 있는 소속 코드입니다. 다른 코드를 정해 주세요.');
      if (key.length < 4) return fail(400, '관리자 비밀번호는 4자 이상으로 정해 주세요.');
      await sql`INSERT INTO groups (code, key_hash) VALUES (${grp}, ${hash(key)})`;
      return res.json({ ok: true });
    }
    if (!g) return fail(404, '없는 소속 코드입니다. 코치에게 코드를 확인해 주세요.');
    const admin = !!key && g.key_hash === hash(key);
    if (key && !admin) return fail(403, '관리자 비밀번호가 맞지 않습니다.');
    const cfgOut = () => (String(q.ver ?? b.ver ?? '') === String(g.cfg_ver) ? {} : { cfg: g.cfg });
    if (a === 'join') return res.json({ ok: true, admin, ver: g.cfg_ver, cfg: g.cfg });
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
    return fail(400, '알 수 없는 요청입니다.');
  } catch (e) { return fail(500, '서버 오류: ' + e.message); }
};
