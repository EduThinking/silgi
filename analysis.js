/* 체대입시 실기 분석 로직 (순수 함수, 화면과 분리) */
(function (root) {
  'use strict';
  const L = { NOSE: 0, SH: [11, 12], EL: [13, 14], WR: [15, 16], HIP: [23, 24], KNEE: [25, 26], ANK: [27, 28], HEEL: [29, 30], TOE: [31, 32] };
  const DEF = {
    jump: { lineTolCm: 3, ref: { knee: [80, 110], armBack: [30, 90], armTO: [110, 170], ext: [160, 180], hipExt: [160, 180], angle: [28, 40], reach: [45, 100], landKnee: [60, 120] } },
    situp: { upMode: 'elbow', upAngle: 70, elbowTol: 0.22, downAngle: 12, limitSec: 60, hipLift: true, startT: null, ref: { validRate: [90, 100], paceDrop: [-100, 20], kneeAngle: [70, 100], hipLiftN: [0, 0] } }
  };

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const deg = r => r * 180 / Math.PI;
  function ang(a, b, c) { // b가 꼭짓점인 내각
    const v1 = [a[0] - b[0], a[1] - b[1]], v2 = [c[0] - b[0], c[1] - b[1]];
    const d = Math.hypot(...v1) * Math.hypot(...v2) || 1e-9;
    return deg(Math.acos(Math.max(-1, Math.min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / d))));
  }
  const median = a => { const s = [...a].sort((x, y) => x - y); return s.length ? (s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2) : NaN; };
  const quant = (a, q) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.max(0, Math.round(q * (s.length - 1))))]; };
  const mean = a => a.reduce((s, v) => s + v, 0) / (a.length || 1);
  const smooth = v => v.map((x, i) => i === 0 || i === v.length - 1 ? x : 0.25 * v[i - 1] + 0.5 * x + 0.25 * v[i + 1]);
  const round = (v, d = 1) => Math.round(v * 10 ** d) / 10 ** d;
  const err = msg => ({ ok: false, error: msg });

  /* 인식 안 된 프레임 보간 + 가벼운 평활 */
  function prep(frames) {
    let a = frames.findIndex(f => f && f.p), b = frames.length - 1;
    while (b >= 0 && !(frames[b] && frames[b].p)) b--;
    if (a < 0 || b <= a) return { n: 0, t: [], p: [] };
    const fr = frames.slice(a, b + 1), t = fr.map(f => f.t), p = fr.map(f => f.p ? f.p.map(q => q.slice()) : null);
    for (let i = 0; i < p.length; i++) if (!p[i]) {
      let j = i; while (!p[j]) j++;
      for (let k = i; k < j; k++) { const w = (t[k] - t[i - 1]) / (t[j] - t[i - 1]); p[k] = p[i - 1].map((q, m) => [q[0] + (p[j][m][0] - q[0]) * w, q[1] + (p[j][m][1] - q[1]) * w, Math.min(q[2], p[j][m][2])]); }
      i = j;
    }
    const s = p.map((f, i) => i === 0 || i === p.length - 1 ? f : f.map((q, m) => [0.25 * p[i - 1][m][0] + 0.5 * q[0] + 0.25 * p[i + 1][m][0], 0.25 * p[i - 1][m][1] + 0.5 * q[1] + 0.25 * p[i + 1][m][1], q[2]]));
    return { n: s.length, t, p: s };
  }
  /* 카메라 쪽(잘 보이는 쪽) 관절 선택: 0=왼쪽, 1=오른쪽 */
  function nearSide(S, keys) {
    const sc = [0, 0];
    S.p.forEach(f => keys.forEach(k => { sc[0] += f[k[0]][2]; sc[1] += f[k[1]][2]; }));
    return sc[1] > sc[0] ? 1 : 0;
  }
  /* 자세와 무관한 신장 추정(픽셀): 몸통+허벅지+정강이 ≈ 신장의 78% */
  function bodyH(S) {
    const sd = nearSide(S, [L.SH, L.HIP, L.KNEE, L.ANK]);
    return median(S.p.map(f => dist(f[L.SH[sd]], f[L.HIP[sd]]) + dist(f[L.HIP[sd]], f[L.KNEE[sd]]) + dist(f[L.KNEE[sd]], f[L.ANK[sd]]))) / 0.78;
  }
  function verdict(v, ref) { return !ref ? 'info' : v < ref[0] ? 'low' : v > ref[1] ? 'high' : 'good'; }
  function metric(key, label, value, unit, ref, d = 0) { return { key, label, value: round(value, d), unit, ref: ref || null, verdict: verdict(value, ref) }; }
  const idxAt = (S, time) => { let b = 0; for (let i = 0; i < S.n; i++) if (Math.abs(S.t[i] - time) < Math.abs(S.t[b] - time)) b = i; return b; };

  /* ───────── 제자리멀리뛰기 ───────── */
  const JUMP_TIPS = {
    'knee:high': { p: '반동이 얕습니다. 앉는 깊이가 부족해 다리 힘을 다 쓰지 못합니다.', fix: '팔을 뒤로 보낼 때 엉덩이를 뒤로 빼며 무릎이 90~110° 될 때까지 빠르게 앉았다가 바로 튀어 오르세요.', ex: ['카운터무브먼트 점프 3×6', '박스 스쿼트 4×5', '스쿼트 점프 3×8'] },
    'knee:low': { p: '너무 깊게 앉습니다. 반동이 끊겨 탄성을 잃습니다.', fix: '앉는 깊이를 조금 줄이고, 내려가는 속도를 높여 멈춤 없이 올라오세요.', ex: ['쿼터 스쿼트 점프 3×6', '뎁스 점프(낮은 박스) 3×5'] },
    'armBack:low': { p: '팔 백스윙이 작습니다.', fix: '앉을 때 양팔을 엉덩이 뒤까지 쭉 뻗어 보내세요. 팔이 뒤로 갈수록 앞으로 던지는 힘이 커집니다.', ex: ['팔 스윙 타이밍 드릴(제자리) 3×10', '밴드 풀 어파트 3×15', '어깨 신전 스트레칭'] },
    'armTO:low': { p: '도약 순간 팔이 충분히 올라오지 않습니다.', fix: '발이 떨어지는 순간 팔이 눈높이 위까지 올라와 있어야 합니다. 팔을 먼저 던지고 다리가 따라간다는 느낌으로 뛰세요.', ex: ['메디신볼 언더핸드 전방 던지기 3×8', '팔 스윙 점프(팔만 강조) 3×6'] },
    'ext:low': { p: '도약 때 무릎이 다 펴지지 않습니다.', fix: '발끝으로 바닥을 끝까지 밀어내세요. 발목·무릎·엉덩이가 일직선이 된 뒤에 발이 떨어져야 합니다.', ex: ['케틀벨 스윙 4×10', '카프 레이즈 4×15', '연속 브로드 점프 3×4'] },
    'hipExt:low': { p: '도약 때 엉덩이가 다 펴지지 않아 상체가 접힌 채로 뜹니다.', fix: '골반을 앞으로 밀어 몸을 길게 펴면서 뛰세요.', ex: ['힙 쓰러스트 4×8', '루마니안 데드리프트 4×6', '케틀벨 스윙 4×10'] },
    'angle:low': { p: '도약 각도가 낮아 체공 시간이 짧습니다.', fix: '시선을 3~4m 앞 바닥이 아니라 정면 위쪽에 두고, 앞쪽 위로 뛰어오르세요.', ex: ['낮은 허들(30cm) 넘어 멀리뛰기 3×5', '박스 점프 3×5'] },
    'angle:high': { p: '도약 각도가 높아 위로만 뜨고 앞으로 나가지 못합니다.', fix: '뛰기 직전 몸 전체를 앞으로 기울여 무게중심이 발끝을 넘어간 뒤에 밀어내세요.', ex: ['전경 자세 낙하 드릴 3×6', '연속 브로드 점프 3×4'] },
    'reach:low': { p: '착지 때 발을 앞으로 내밀지 못합니다. 여기서 10~20cm를 잃습니다.', fix: '공중에서 무릎을 가슴으로 당긴 뒤 착지 직전 발뒤꿈치를 앞으로 던지세요. 팔은 뒤로 내립니다.', ex: ['행잉 니 레이즈 3×10', '턱 점프 3×6', '파이크 점프 3×5', '햄스트링 스트레칭'] },
    'landKnee:high': { p: '착지가 뻣뻣합니다. 뒤로 넘어지거나 부상 위험이 있습니다.', fix: '뒤꿈치가 닿자마자 무릎을 굽혀 엉덩이를 발 쪽으로 당겨 오세요.', ex: ['박스 드랍 착지 3×6', '스쿼트 홀드 3×30초'] }
  };

  function analyzeJump(frames, o) {
    o = o || {};
    const cfg = Object.assign({}, DEF.jump, o.cfg, { ref: Object.assign({}, DEF.jump.ref, o.cfg && o.cfg.ref) });
    const S = prep(frames);
    if (S.n < 12) return err('사람이 인식된 구간이 너무 짧습니다. 전신이 옆모습으로 나오게 다시 촬영해 주세요.');
    const H = bodyH(S), ppc = o.pxPerCm || (o.heightCm ? H / o.heightCm : 0);
    if (!ppc) return err('거리 기준이 없습니다. 기준점을 찍거나 키를 입력해 주세요.');
    const k = o.ground ? o.ground.k : 0, slopeF = Math.sqrt(1 + k * k);
    const T = S.t, P = S.p, n = S.n;
    const feet = f => [f[29], f[30], f[31], f[32]];
    const lowY = P.map(f => Math.max(...feet(f).map(q => q[1])));
    const footX = P.map(f => (f[29][0] + f[30][0]) / 2);
    const raw = lowY.map((y, i) => k * footX[i] - y), base = quant(raw, 0.2);
    const h = raw.map(v => v - base), thr = 0.04 * H;
    // 체공 구간 찾기
    const runs = []; let s = -1;
    for (let i = 0; i <= n; i++) {
      const air = i < n && h[i] > thr;
      if (air && s < 0) s = i;
      if (!air && s >= 0) { if (runs.length && s - runs[runs.length - 1].e <= 2) runs[runs.length - 1].e = i - 1; else runs.push({ s, e: i - 1 }); s = -1; }
    }
    let main = null, best = 0;
    runs.forEach(r => { const a = Math.max(0, r.s - 1), b = Math.min(n - 1, r.e + 1), dx = Math.abs(footX[b] - footX[a]); if (T[b] - T[a] >= 0.12 && dx >= 0.3 * H && dx > best) { best = dx; main = r; } });
    if (!main) return err('도약 구간을 찾지 못했습니다. 준비 자세부터 착지 후 1초까지 발이 화면에 다 나오는지 확인해 주세요.');
    if (main.s < 2 || main.e > n - 3) return err('도약 앞뒤 장면이 부족합니다. 준비 자세부터 착지 후 1초까지 담아 주세요.');
    const a = main.s - 1, b = main.e + 1, dir = Math.sign(footX[b] - footX[a]) || 1;
    // 구름 발끝 위치
    const toeF = i => Math.max(P[i][31][0] * dir, P[i][32][0] * dir);
    const heelR = i => Math.min(P[i][29][0] * dir, P[i][30][0] * dir);
    const toeX = median([a - 3, a - 2, a - 1, a].filter(i => i >= 0).map(toeF));
    const hasLine = !!(o.line && isFinite(o.line.x)), lineX = hasLine ? o.line.x * dir : toeX;
    // 착지 뒤꿈치: 발이 멈춘 첫 프레임
    let j = b; while (j < Math.min(n - 2, b + 5) && Math.abs(heelR(j + 1) - heelR(j)) > 0.012 * H) j++;
    let landX = heelR(j);
    const fouls = [], notes = [];
    // 착지 후 1초: 뒤로 물러남 / 손·엉덩이 짚기
    const endI = idxAt(S, T[b] + 1.0);
    let backX = landX, backWhy = null;
    for (let i = j; i <= endI; i++) {
      if (h[i] <= thr && heelR(i) < backX - 0.04 * H) { backX = heelR(i); backWhy = '발'; }
      [15, 16].forEach(w => { const q = P[i][w], hh = k * q[0] - q[1] - base; if (q[2] > 0.3 && hh < 0.05 * H && q[0] * dir < backX - 0.03 * H) { backX = q[0] * dir; backWhy = '손'; } });
      const hp = mid(P[i][23], P[i][24]), hh = k * hp[0] - hp[1] - base;
      if (hh < 0.09 * H && hp[0] * dir < backX) { backX = hp[0] * dir; backWhy = '엉덩이'; }
    }
    if (backWhy) { fouls.push({ code: 'back', sev: 'bad', label: `착지 후 ${backWhy}이 뒤에 닿음`, detail: `가장 뒤쪽 접지점 기준으로 ${round((landX - backX) * slopeF / ppc)}cm 줄어듭니다.` }); landX = backX; }
    const distCm = (landX - lineX) * slopeF / ppc;
    if (hasLine) {
      const over = (toeX - lineX) * slopeF / ppc;
      if (over > cfg.lineTolCm) fouls.push({ code: 'line', sev: 'bad', label: '구름선 밟음(넘음)', detail: `발끝이 선을 약 ${round(over)}cm 넘었습니다.` });
      else notes.push(`구름선까지 여유 ${round(Math.max(0, -over))}cm`);
    } else notes.push('구름선을 지정하지 않아 발끝부터 잰 거리입니다. 선 밟기 판정은 하지 않았습니다.');
    // 이중 도약 / 준비 중 발 이동
    const w0 = idxAt(S, T[a] - 1.5);
    if (runs.some(r => r !== main && r.e < main.s && T[r.s] >= T[w0] && T[r.e] - T[r.s] >= 0.05)) fouls.push({ code: 'double', sev: 'warn', label: '이중 도약 의심', detail: '도약 직전에 발이 한 번 떴습니다. 영상을 확인하세요.' });
    else if (Math.abs(toeF(w0) - toeX) > 0.06 * H) fouls.push({ code: 'shift', sev: 'warn', label: '준비 중 발 이동 의심', detail: '도약 전 1.5초 안에 발 위치가 바뀌었습니다.' });

    // 자세 지표
    const sd = nearSide(S, [L.HIP, L.KNEE, L.ANK, L.SH]);
    const J = (i, key) => P[i][L[key][sd]];
    const knee = i => ang(J(i, 'HIP'), J(i, 'KNEE'), J(i, 'ANK'));
    const hipA = i => ang(J(i, 'SH'), J(i, 'HIP'), J(i, 'KNEE'));
    const arm = i => { // 몸통 기준 팔 각도, 앞쪽(진행 방향)이 +
      const sh = J(i, 'SH'), hp = J(i, 'HIP'), wr = J(i, 'WR'), t = [hp[0] - sh[0], hp[1] - sh[1]], v = [wr[0] - sh[0], wr[1] - sh[1]];
      return -deg(Math.atan2(t[0] * v[1] - t[1] * v[0], t[0] * v[0] + t[1] * v[1])) * dir;
    };
    let deep = w0; for (let i = w0; i <= a; i++) if (knee(i) < knee(deep)) deep = i;
    let armBack = 0; for (let i = w0; i <= a; i++) armBack = Math.max(armBack, -arm(i));
    let armTO = arm(main.s); if (armTO < -90) armTO += 360;
    const ext = Math.max(...[a - 1, a, main.s].map(knee)), hipExt = Math.max(...[a - 1, a, main.s].map(hipA));
    const com = i => { const sh = mid(P[i][11], P[i][12]), hp = mid(P[i][23], P[i][24]); return mid(sh, hp); };
    const Tf = T[b] - T[a], vx = Math.abs(com(b)[0] - com(a)[0]) / ppc / 100 / Tf;
    const vy = ((com(a)[1] - com(b)[1]) / ppc / 100 + 4.905 * Tf * Tf) / Tf;
    const leg = dist(J(b, 'HIP'), J(b, 'KNEE')) + dist(J(b, 'KNEE'), J(b, 'ANK'));
    const hipX = mid(P[j][23], P[j][24])[0] * dir, reach = (heelR(j) - hipX) / (median(P.map((f, i) => dist(J(i, 'HIP'), J(i, 'KNEE')) + dist(J(i, 'KNEE'), J(i, 'ANK')))) || leg) * 100;
    const le = idxAt(S, T[b] + 0.5); let landKnee = 180; for (let i = b; i <= le; i++) landKnee = Math.min(landKnee, knee(i));
    const R = cfg.ref;
    const metrics = [
      metric('knee', '반동 깊이 (가장 깊이 앉았을 때 무릎 각도)', knee(deep), '°', R.knee),
      metric('armBack', '팔 백스윙 (몸통 뒤로 보낸 각도)', armBack, '°', R.armBack),
      metric('armTO', '도약 순간 팔 높이 (몸통 기준)', armTO, '°', R.armTO),
      metric('ext', '도약 순간 무릎 펴짐', ext, '°', R.ext),
      metric('hipExt', '도약 순간 엉덩이 펴짐', hipExt, '°', R.hipExt),
      metric('angle', '도약 각도', deg(Math.atan2(vy, vx)), '°', R.angle),
      metric('reach', '착지 시 발 내밀기 (다리 길이 대비)', reach, '%', R.reach),
      metric('landKnee', '착지 무릎 굽힘 (최소 각도)', landKnee, '°', R.landKnee),
      metric('speed', '도약 속도', Math.hypot(vx, vy), 'm/s', null, 2),
      metric('flight', '체공 시간', Tf, '초', null, 2)
    ];
    return {
      ok: true, event: 'jump', record: { value: round(distCm), unit: 'cm', valid: !fouls.some(f => f.sev === 'bad') }, fouls, notes, metrics,
      tips: tipsFor(metrics, JUMP_TIPS),
      events: [{ t: T[deep], label: '최대 반동' }, { t: T[main.s], label: '도약' }, { t: T[j], label: '착지' }],
      anchor: T[main.s], geom: { base, dir, lineX: lineX * dir, landX: landX * dir, toeX: toeX * dir, k, hasLine }, pxPerCm: ppc, hPx: H
    };
  }

  function tipsFor(metrics, table) {
    const out = [];
    metrics.forEach(m => { const t = table[m.key + ':' + m.verdict]; if (t) out.push(Object.assign({ key: m.key, label: m.label }, t)); });
    return out;
  }

  /* ───────── 윗몸일으키기 ───────── */
  function pivots(v, delta) {
    const out = []; let hi = 0, lo = 0, d = 0;
    for (let i = 1; i < v.length; i++) {
      if (v[i] > v[hi]) hi = i; if (v[i] < v[lo]) lo = i;
      if (d >= 0 && v[hi] - v[i] >= delta) { out.push({ i: hi, type: 'hi' }); d = -1; lo = i; }
      else if (d <= 0 && v[i] - v[lo] >= delta) { out.push({ i: lo, type: 'lo' }); d = 1; hi = i; }
    }
    if (d === 1 && v[hi] - v[out[out.length - 1].i] >= delta) out.push({ i: hi, type: 'hi' });
    if (d === -1) out.push({ i: lo, type: 'lo' });
    return out;
  }
  function segDist(p, a, b) {
    const vx = b[0] - a[0], vy = b[1] - a[1], w = Math.max(0, Math.min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy || 1e-9)));
    return Math.hypot(p[0] - a[0] - w * vx, p[1] - a[1] - w * vy);
  }
  function analyzeSitup(frames, o) {
    o = o || {};
    const cfg = Object.assign({}, DEF.situp, o.cfg, { ref: Object.assign({}, DEF.situp.ref, o.cfg && o.cfg.ref) });
    const S = prep(frames);
    if (S.n < 20) return err('사람이 인식된 구간이 너무 짧습니다. 옆에서 전신이 나오게 촬영해 주세요.');
    const T = S.t, P = S.p, n = S.n, sd = nearSide(S, [L.SH, L.HIP, L.KNEE]);
    const J = (i, key) => P[i][L[key][sd]];
    const th = smooth(P.map((f, i) => deg(Math.atan2(J(i, 'HIP')[1] - J(i, 'SH')[1], Math.abs(J(i, 'SH')[0] - J(i, 'HIP')[0])))));
    const thigh = median(P.map((f, i) => dist(J(i, 'HIP'), J(i, 'KNEE'))));
    const piv = pivots(th, 25);
    if (piv.filter(p => p.type === 'hi').length < 1) return err('윗몸일으키기 동작을 찾지 못했습니다. 카메라를 몸 옆에 두고 상체와 무릎이 다 나오게 찍어 주세요.');
    const elbowGap = i => { let m = 1e9; for (const e of [13, 14]) for (const s2 of [0, 1]) { const kn = P[i][L.KNEE[s2]], hp = P[i][L.HIP[s2]]; m = Math.min(m, segDist(P[i][e], kn, mid(kn, hp))); } return m / thigh; };
    const troughs = piv.filter(p => p.type === 'lo').map(p => P[p.i]), hipBase = median((troughs.length ? piv.filter(p => p.type === 'lo').map(p => J(p.i, 'HIP')[1]) : [J(0, 'HIP')[1]]));
    const reps = []; let prevLo = 0, startT = cfg.startT;
    piv.forEach((p, pi) => {
      if (p.type === 'lo') { prevLo = p.i; return; }
      const lo = prevLo, w = 0.3; let gap = 1e9, hipUp = 0, rise = lo;
      for (let i = lo; i <= p.i; i++) { hipUp = Math.max(hipUp, hipBase - J(i, 'HIP')[1]); if (rise === lo && th[i] > th[lo] + 8) rise = i; }
      for (let i = idxAt(S, T[p.i] - w); i <= idxAt(S, T[p.i] + w); i++) gap = Math.min(gap, elbowGap(i));
      if (startT == null) startT = T[Math.max(lo, rise - 1)];
      const upOk = cfg.upMode === 'elbow' ? gap <= cfg.elbowTol : th[p.i] >= cfg.upAngle;
      const downOk = th[lo] <= cfg.downAngle, lift = hipUp > 0.15 * thigh;
      let reason = null;
      if (T[p.i] < startT) reason = '시작 전';
      else if (T[p.i] > startT + cfg.limitSec) reason = '시간 초과';
      else if (!downOk) reason = '어깨가 바닥에 안 닿음';
      else if (!upOk) reason = cfg.upMode === 'elbow' ? '팔꿈치가 무릎에 안 닿음' : '상체 각도 부족';
      else if (lift && cfg.hipLift) reason = '엉덩이 들림';
      reps.push({ tStart: T[lo], t: T[p.i], valid: !reason, reason, peak: round(th[p.i]), down: round(th[lo]), gap: round(gap, 2), lift });
    });
    const inTime = reps.filter(r => r.reason !== '시작 전' && r.reason !== '시간 초과');
    const valid = inTime.filter(r => r.valid); let c = 0; reps.forEach(r => { r.n = r.valid ? ++c : null; });
    const reasons = {}; inTime.filter(r => !r.valid).forEach(r => { reasons[r.reason] = (reasons[r.reason] || 0) + 1; });
    const durs = inTime.slice(1).map((r, i) => r.t - inTime[i].t), third = Math.max(1, Math.floor(durs.length / 3));
    const paceDrop = durs.length >= 6 ? (mean(durs.slice(-third)) / mean(durs.slice(0, third)) - 1) * 100 : 0;
    const kneeA = median(P.map((f, i) => ang(J(i, 'HIP'), J(i, 'KNEE'), J(i, 'ANK'))));
    const buckets = []; inTime.forEach(r => { const bI = Math.floor((r.t - startT) / 10); while (buckets.length <= bI) buckets.push({ valid: 0, invalid: 0 }); buckets[bI][r.valid ? 'valid' : 'invalid']++; });
    const R = cfg.ref, liftN = inTime.filter(r => r.lift).length;
    const metrics = [
      metric('validRate', '유효율 (인정 개수 ÷ 시도 개수)', inTime.length ? valid.length / inTime.length * 100 : 0, '%', R.validRate),
      metric('paceDrop', '후반 페이스 저하 (처음 대비 1회 시간 증가)', paceDrop, '%', R.paceDrop),
      metric('kneeAngle', '무릎 각도', kneeA, '°', R.kneeAngle),
      metric('hipLiftN', '엉덩이 들림 횟수', liftN, '회', R.hipLiftN),
      metric('repTime', '평균 1회 시간', durs.length ? mean(durs) : 0, '초', null, 2),
      metric('peakAngle', '평균 최고 상체 각도', mean(inTime.map(r => r.peak)), '°', null)
    ];
    const tips = [];
    const up = (reasons['팔꿈치가 무릎에 안 닿음'] || 0) + (reasons['상체 각도 부족'] || 0);
    if (up) tips.push({ key: 'up', label: `끝까지 안 올라온 횟수 ${up}회`, p: '상체를 끝까지 올리지 못해 무효가 됩니다.', fix: '턱을 당기고 가슴이 허벅지에 닿는다는 느낌으로 올라오세요. 개수보다 접촉을 먼저 맞추고 속도를 올립니다.', ex: ['싯업 상단 2초 정지 3×12', 'V-업 3×10', '행잉 레그 레이즈 3×8', '장요근 밴드 니 드라이브 3×12'] });
    if (reasons['어깨가 바닥에 안 닿음']) tips.push({ key: 'down', label: `어깨 미접촉 ${reasons['어깨가 바닥에 안 닿음']}회`, p: '내려갈 때 어깨가 바닥에 닿기 전에 다시 올라옵니다.', fix: '등 윗부분이 매트를 찍는 감각을 확인하고 올라오세요. 메트로놈(분당 50~60회)에 맞춰 연습하면 리듬이 잡힙니다.', ex: ['템포 싯업(1초 업·1초 다운) 3×20', '할로우 바디 록 3×15'] });
    if (liftN) tips.push({ key: 'hip', label: `엉덩이 들림 ${liftN}회`, p: '반동으로 올라오면서 엉덩이가 뜹니다.', fix: '엉덩이를 매트에 붙이고 복근으로만 말아 올리세요.', ex: ['데드버그 3×12', '할로우 바디 홀드 3×30초', '리버스 크런치 3×15'] });
    if (paceDrop > R.paceDrop[1]) tips.push({ key: 'pace', label: `후반 페이스 ${round(paceDrop)}% 저하`, p: '후반으로 갈수록 속도가 크게 떨어집니다.', fix: '처음 20초를 최고 속도의 90%로 시작해 끝까지 유지하는 페이스를 찾으세요.', ex: ['타바타 싯업(20초 수행·10초 휴식 ×8)', '1분 싯업 ×3세트(휴식 2분)', '플랭크 3×60초'] });
    if (kneeA > R.kneeAngle[1] || kneeA < R.kneeAngle[0]) tips.push({ key: 'knee', label: `무릎 각도 ${round(kneeA)}°`, p: '무릎 각도가 기준(약 90°)에서 벗어났습니다.', fix: '발뒤꿈치와 엉덩이 사이를 한 뼘 반 정도로 맞추고 시작하세요.', ex: [] });
    return {
      ok: true, event: 'situp', record: { value: valid.length, unit: '개', valid: true }, attempts: inTime.length, reasons, reps, buckets, metrics, tips,
      fouls: Object.keys(reasons).map(r => ({ code: r, sev: 'warn', label: `${r} ${reasons[r]}회`, detail: '무효 처리했습니다.' })),
      notes: [`시도 ${inTime.length}회 중 ${valid.length}회 인정`, `측정 시작 ${round(startT, 2)}초, 제한 ${cfg.limitSec}초`],
      events: reps.filter(r => r.reason !== '시작 전').map(r => ({ t: r.t, label: r.valid ? String(r.n) : '×', bad: !r.valid })), anchor: startT, series: { t: T, v: th.map(x => round(x)) }
    };
  }

  const api = { DEF, prep, bodyH, analyzeJump, analyzeSitup, pivots, ang };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Analysis = api;
})(typeof self !== 'undefined' ? self : this);
