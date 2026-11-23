/* 체대입시 실기 분석 로직 (순수 함수, 화면과 분리) */
(function (root) {
  'use strict';
  const L = { NOSE: 0, SH: [11, 12], EL: [13, 14], WR: [15, 16], HIP: [23, 24], KNEE: [25, 26], ANK: [27, 28], HEEL: [29, 30], TOE: [31, 32] };
  const DEF = {
    // shoeCm: 관절 인식의 뒤꿈치·발끝 점은 신발 실제 끝보다 안쪽에 찍힌다. 테이프 4점 방식에서 실측 기록 5건(190·193·249·250·257cm)에 맞춰 잡은 보정값.
    jump: { calibMode: 'height', lineTolCm: 3, shoeCm: 8.5, ref: { knee: [80, 110], armBack: [30, 90], armTO: [110, 180], ext: [160, 180], hipExt: [160, 180], angle: [25, 39], speed: [2.6, 4.3], tuck: [0, 100], tuckT: [0, 60], heelKick: [-200, 10], ankle: [60, 115], legOut: [0.03, 0.15], tdKnee: [125, 180], tdHip: [20, 100], reach: [45, 100], landKnee: [30, 120] } },
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
  function metric(key, label, value, unit, ref, d = 0, at, j) { const v = round(value, d); return { key, label, value: v, unit, ref: ref || null, verdict: verdict(v, ref), at: at == null || !isFinite(at) ? null : round(at, 3), j: j || null }; }
  /* 바닥 원근 보정: 출발선·거리선 테이프의 [가까운 끝, 먼 끝] 두 쌍(4점)으로 바닥 위 거리(cm)를 구한다.
     두 선이 화면에서 만나는 소실점을 지나는 직선은 바닥에서 출발선과 나란한 선이다. 카메라를 정확히 옆에 두면
     (u-소실점x)/(v-소실점y) 값이 실제 거리에 비례하므로, 카메라에서 가깝든 멀든 같은 기준으로 잴 수 있다. */
  function floorModel(pts, cm) {
    if (!pts || pts.length < 4 || !(cm > 0)) return null;
    const [a, b, c, d] = pts, D = (a.x - b.x) * (c.y - d.y) - (a.y - b.y) * (c.x - d.x);
    if (Math.abs(D) < 1e-6) return null;
    const t = ((a.x - c.x) * (c.y - d.y) - (a.y - c.y) * (c.x - d.x)) / D, V = { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
    if (!(V.y < Math.min(a.y, b.y, c.y, d.y) - 20)) return null; // 소실점이 바닥보다 위에 있어야 한다
    const q = (u, v) => (u - V.x) / (v - V.y), q1 = q(a.x, a.y), q2 = q(c.x, c.y);
    if (Math.abs(q2 - q1) < 1e-6) return null;
    const K = cm / (q2 - q1), X = (u, v) => K * (q(u, v) - q1);
    X.vp = V; X.ppc = Math.hypot(c.x - a.x, c.y - a.y) / cm; X.fromStart = (y) => ({ x: V.x + q1 * (y - V.y) }); X.at = (cmv, y) => V.x + (cmv / K + q1) * (y - V.y);
    return X;
  }
  const idxAt = (S, time) => { let b = 0; for (let i = 0; i < S.n; i++) if (Math.abs(S.t[i] - time) < Math.abs(S.t[b] - time)) b = i; return b; };

  /* ───────── 제자리멀리뛰기 ───────── */
  const JUMP_TIPS = {
    'knee:high': { p: '뛰기 전에 덜 앉습니다. 그래서 다리 힘을 다 쓰지 못합니다.', fix: '팔을 뒤로 보내면서 엉덩이를 뒤로 빼고, 의자에 앉듯이 무릎이 직각쯤 될 때까지 빠르게 앉았다가 멈추지 말고 바로 뛰세요.', ex: ['카운터무브먼트 점프 3×6', '박스 스쿼트 4×5', '스쿼트 점프 3×8'] },
    'knee:low': { p: '너무 깊게 앉습니다. 앉았다 일어나는 흐름이 끊겨서 튀어 오르는 힘이 약해집니다.', fix: '조금 덜 앉고, 더 빠르게 앉았다가 멈추지 말고 바로 올라오세요.', ex: ['쿼터 스쿼트 점프 3×6', '뎁스 점프(낮은 박스) 3×5'] },
    'armBack:low': { p: '팔 백스윙이 작습니다.', fix: '앉을 때 양팔을 엉덩이 뒤까지 쭉 뻗어 보내세요. 팔이 뒤로 갈수록 앞으로 던지는 힘이 커집니다.', ex: ['팔 스윙 타이밍 드릴(제자리) 3×10', '밴드 풀 어파트 3×15', '어깨 신전 스트레칭'] },
    'armTO:low': { p: '뛰는 순간 팔이 덜 올라옵니다.', fix: '발이 떨어지는 순간 팔이 눈높이 위까지 올라와 있어야 합니다. 팔을 먼저 던지고 다리가 따라간다는 느낌으로 뛰세요.', ex: ['메디신볼 언더핸드 전방 던지기 3×8', '팔 스윙 점프(팔만 강조) 3×6'] },
    'ext:low': { p: '무릎이 다 펴지기 전에 발이 땅에서 떨어집니다.', fix: '발끝으로 바닥을 끝까지 밀어내세요. 발목·무릎·엉덩이가 일직선이 된 뒤에 발이 떨어져야 합니다.', ex: ['케틀벨 스윙 4×10', '카프 레이즈 4×15', '연속 브로드 점프 3×4'] },
    'hipExt:low': { p: '뛸 때 몸이 다 펴지지 않아 상체를 숙인 채로 뜹니다.', fix: '배를 앞으로 내민다는 느낌으로 몸을 길게 쭉 펴면서 뛰세요.', ex: ['힙 쓰러스트 4×8', '루마니안 데드리프트 4×6', '케틀벨 스윙 4×10'] },
    'speed:low': { p: '발이 땅에서 떨어지는 순간의 속도가 느립니다. 기록에 가장 큰 영향을 주는 부분입니다.', fix: '앉았다 일어나는 동작을 끊지 말고 한 번에 빠르게 하세요. 팔을 앞으로 던지는 것과 다리로 바닥을 미는 것을 같은 순간에 해야 합니다. 팔만 잘 써도 기록이 10~20% 늘어납니다.', ex: ['스쿼트 점프 4×5', '케틀벨 스윙 4×10', '메디신볼 전방 던지기 3×8', '연속 브로드 점프 3×4'] },
    'angle:low': { p: '너무 낮게 뜁니다. 공중에 떠 있는 시간이 짧아서 멀리 가지 못합니다.', fix: '시선을 3~4m 앞 바닥이 아니라 정면 위쪽에 두고, 앞쪽 위로 뛰어오르세요.', ex: ['낮은 허들(30cm) 넘어 멀리뛰기 3×5', '박스 점프 3×5'] },
    'angle:high': { p: '너무 위로 뜁니다. 높이만 뜨고 앞으로 나가지 못합니다.', fix: '뛰기 직전에 몸 전체를 앞으로 기울여서, 몸이 발끝보다 앞으로 넘어간 뒤에 바닥을 밀어내세요. 각도는 조금 달라도 기록 차이가 2cm 정도뿐이라, 속도와 착지를 먼저 고치는 게 좋습니다.', ex: ['전경 자세 낙하 드릴 3×6', '연속 브로드 점프 3×4'] },
    'tuck:high': { p: '공중에서 몸을 접지 못하고 편 채로 떨어집니다.', fix: '발이 떨어진 뒤 무릎을 가슴 쪽으로 당겨 몸을 접고, 착지 직전에 다리를 앞으로 뻗으세요. 접었다 펴야 발이 멀리 나갑니다.', ex: ['턱 점프 3×6', '행잉 니 레이즈 3×10', 'V-업 3×10'] },
    'tuckT:high': { p: '무릎을 늦게 가져옵니다. 내려오는 중에야 다리가 앞으로 나옵니다.', fix: '발이 떨어지자마자 무릎을 가슴 쪽으로 끌어올리세요. 최고점에 닿기 전에 무릎이 다 올라와 있어야 착지 준비할 시간이 남습니다.', ex: ['턱 점프 3×6', '행잉 니 레이즈 3×10', '장요근 밴드 니 드라이브 3×12'] },
    'heelKick:high': { p: '공중에서 발이 뒤로 차올려져 발바닥이 하늘을 봅니다. 다리를 앞으로 가져오는 길이 멀어집니다.', fix: '뒤꿈치를 엉덩이로 차지 말고 무릎을 앞·위로 들어 올리세요. 발끝은 정강이 쪽으로 당겨 둡니다.', ex: ['턱 점프 3×6', '행잉 니 레이즈 3×10'] },
    'ankle:high': { p: '착지 직전에 발끝이 아래로 늘어져 있습니다. 발끝이 먼저 닿으면 거리를 잃고 앞으로 고꾸라집니다.', fix: '다리를 뻗을 때 발끝을 몸 쪽으로 당겨 뒤꿈치부터 닿게 하세요.', ex: ['앉아서 무릎 누르고 발끝 당기기 3×20초', '파이크 점프 3×5'] },
    'legOut:high': { p: '다리를 너무 일찍 뻗습니다. 발이 먼저 내려가 땅에 일찍 닿습니다.', fix: '몸을 접은 자세를 조금 더 유지하다가 땅에 닿기 직전에 뒤꿈치를 앞으로 던지세요.', ex: ['턱 점프 후 늦게 펴기 3×5', '파이크 점프 3×5'] },
    'legOut:low': { p: '착지 전에 다리를 앞으로 뻗지 못합니다.', fix: '내려오면서 무릎을 펴 뒤꿈치를 앞으로 내밀고, 동시에 팔을 뒤로 내리세요. 팔이 뒤로 가면 다리가 앞으로 나옵니다.', ex: ['파이크 점프 3×5', 'V-업 3×10', '햄스트링 스트레칭'] },
    'tdKnee:low': { p: '닿는 순간 무릎이 이미 굽어 있어 발이 몸 가까이 떨어집니다.', fix: '닿는 순간까지는 무릎을 펴서 뒤꿈치를 멀리 보내고, 닿은 뒤에 굽혀 받으세요.', ex: ['파이크 점프 3×5', '햄스트링 스트레칭'] },
    'tdHip:high': { p: '땅에 닿는 순간 상체가 서 있습니다. 상체를 숙여야 발이 몸보다 앞에 떨어집니다.', fix: '착지 직전에 가슴을 허벅지 쪽으로 숙이고 팔을 뒤로 내리세요.', ex: ['V-업 3×10', '행잉 레그 레이즈 3×8'] },
    'reach:low': { p: '착지 때 발을 앞으로 내밀지 못합니다. 여기서 10~20cm를 잃습니다.', fix: '공중에서 무릎을 가슴으로 당긴 뒤 착지 직전 발뒤꿈치를 앞으로 던지세요. 팔은 뒤로 내립니다.', ex: ['행잉 니 레이즈 3×10', '턱 점프 3×6', '파이크 점프 3×5', '햄스트링 스트레칭'] },
    'landKnee:high': { p: '무릎을 거의 굽히지 않고 착지합니다. 뒤로 넘어지거나 다칠 수 있습니다.', fix: '뒤꿈치가 닿자마자 무릎을 굽혀 엉덩이를 발 쪽으로 당겨 오세요.', ex: ['박스 드랍 착지 3×6', '스쿼트 홀드 3×30초'] }
  };

  function analyzeJump(frames, o) {
    o = o || {};
    const cfg = Object.assign({}, DEF.jump, o.cfg, { ref: Object.assign({}, DEF.jump.ref, o.cfg && o.cfg.ref) });
    const S = prep(frames);
    if (S.n < 12) return err('사람이 인식된 구간이 너무 짧습니다. 전신이 옆모습으로 나오게 다시 촬영해 주세요.');
    const FM = o.floor ? floorModel(o.floor.pts, o.floor.cm) : null;
    if (o.floor && !FM) return err('테이프 네 점으로 바닥을 맞추지 못했습니다. 출발선과 거리선 테이프의 가까운 끝·먼 끝을 각각 다시 찍어 주세요. 두 선이 화면에서 위쪽으로 모여야 합니다.');
    const H = bodyH(S), ppc = FM ? FM.ppc : o.pxPerCm || (o.heightCm ? H / o.heightCm : 0);
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
    if (!main) return err('뛰는 장면을 찾지 못했습니다. 준비 자세부터 착지 후 1초까지 발이 화면에 다 나오는지 확인해 주세요.');
    if (main.s < 2 || main.e > n - 3) return err('뛰기 전후 장면이 부족합니다. 준비 자세부터 착지 후 1초까지 담아 주세요.');
    const a = main.s - 1, b = main.e + 1, dir = Math.sign(footX[b] - footX[a]) || 1;
    // 구름 발끝 위치
    // 카메라가 비스듬하면 두 발이 화면에서 앞뒤로 어긋나 보인다. 출발 때 두 발끝이 같은 선에 있다고 보고 발마다 어긋난 만큼 보정한다.
    const stance = [a - 3, a - 2, a - 1, a].filter(i => i >= 0), toeS = [0, 1].map(f => median(stance.map(i => P[i][31 + f][0] * dir))), visF = [0, 1].map(f => median(stance.map(i => Math.min(P[i][31 + f][2], P[i][29 + f][2]))));
    const hasLine = !!(o.line && isFinite(o.line.x));
    const fA = hasLine ? (Math.abs(toeS[0] - o.line.x * dir) <= Math.abs(toeS[1] - o.line.x * dir) ? 0 : 1) : (toeS[0] >= toeS[1] ? 0 : 1);
    const feetUse = [0, 1].filter(f => f === fA || visF[f] > 0.5), offF = [0, 1].map(f => toeS[fA] - toeS[f]);
    const toeF = i => P[i][31 + fA][0] * dir;
    const heelR = i => Math.min(...feetUse.map(f => P[i][29 + f][0] * dir + offF[f]));
    const toeX = toeS[fA], lineX = hasLine ? o.line.x * dir : toeX;
    // 착지 직후 발이 멈춘 자리 (자세 지표·뒤 짚기 판정에 쓴다)
    let j = b; while (j < Math.min(n - 2, b + 5) && Math.abs(heelR(j + 1) - heelR(j)) > 0.012 * H) j++;
    const settle = idxAt(S, T[b] + 0.5), settleI = []; let landX = heelR(j); for (let i = j; i <= settle; i++) if (h[i] <= 2.5 * thr) { landX = Math.min(landX, heelR(i)); settleI.push(i); } // 깊이 앉으면 발 관절이 화면에서 조금 떠 보여 여유를 둔다
    const fouls = [], notes = [];
    // 기록 자리: 착지 후 똑바로 섰을 때의 뒤꿈치 (무릎·엉덩이가 펴진 채 0.2초 이상)
    const sdU = nearSide(S, [L.HIP, L.KNEE, L.ANK, L.SH]);
    const settleX = landX, okFeet = i => feetUse.every(f => Math.min(P[i][29 + f][2], P[i][31 + f][2]) >= 0.5) && Math.abs(heelR(i) - settleX) < 0.3 * H;
    const upright = i => okFeet(i) && ang(P[i][L.HIP[sdU]], P[i][L.KNEE[sdU]], P[i][L.ANK[sdU]]) >= 155 && ang(P[i][L.SH[sdU]], P[i][L.HIP[sdU]], P[i][L.KNEE[sdU]]) >= 150 && h[i] <= 2.5 * thr;
    const need = Math.max(5, Math.round(0.2 / ((T[n - 1] - T[0]) / (n - 1) || 0.033)));
    let st = [], cur = [];
    for (let i = j + 1; i < n; i++) { if (upright(i)) { cur.push(i); if (cur.length >= need && !st.length) st = cur; } else { if (st.length) break; cur = []; } }
    const standI = st.length ? st : null;
    // 테이프 4점 방식: 발마다 바닥에 닿는 높이에서 원근을 풀어 cm로 바꾼다
    const vF = (i, f) => Math.max(P[i][29 + f][1], P[i][31 + f][1]);
    const heelC = i => Math.min(...feetUse.map(f => FM(P[i][29 + f][0], vF(i, f))));
    // 착지 후 1초: 뒤로 물러남 / 손·엉덩이 짚기
    const endI = idxAt(S, T[b] + 1.0);
    let backX = landX, backWhy = null, backPt = null;
    for (let i = settle; i <= endI; i++) {
      if (!standI && h[i] <= 2.5 * thr && heelR(i) < backX - 0.06 * H) { backX = heelR(i); backWhy = '발'; backPt = { i }; }
      [15, 16].forEach(w => { const q = P[i][w], hh = k * q[0] - q[1] - base; if (q[2] > 0.3 && hh < 0.05 * H && q[0] * dir < backX - 0.03 * H) { backX = q[0] * dir; backWhy = '손'; backPt = { u: q[0], v: q[1] }; } });
      const hp = mid(P[i][23], P[i][24]), hh = k * hp[0] - hp[1] - base;
      if (hh < 0.09 * H && hp[0] * dir < backX) { backX = hp[0] * dir; backWhy = '엉덩이'; backPt = { u: hp[0], v: Math.max(vF(i, 0), vF(i, 1)) }; }
    }
    let distCm, startCm = 0, landCm = 0, yF = 0;
    const skew = FM ? 0 : Math.abs(offF[1 - fA]) / ppc;
    if (FM) {
      const toeC = [0, 1].map(f => median(stance.map(i => FM(P[i][31 + f][0], vF(i, f)))));
      const fS = toeC[0] >= toeC[1] ? 0 : 1; startCm = toeC[fS];
      const visI = settleI.filter(i => feetUse.every(f => Math.min(P[i][29 + f][2], P[i][31 + f][2]) >= 0.5)), useI = standI || (visI.length ? visI : settleI.length ? settleI : [j]);
      landCm = median(useI.map(heelC)); yF = median(useI.map(i => Math.max(vF(i, 0), vF(i, 1))));
      if (backWhy) {
        const bc = backPt.i != null ? heelC(backPt.i) : FM(backPt.u, backPt.v);
        if (bc < landCm) { fouls.push({ code: 'back', sev: 'bad', label: `착지 후 ${backWhy}이 뒤에 닿음`, detail: `가장 뒤에 닿은 곳부터 재서 기록이 ${round(landCm - bc)}cm 줄어듭니다.` }); landCm = bc; }
      }
      distCm = landCm - startCm - (cfg.shoeCm || 0);
      // 관절 점의 흔들림이 ±3cm쯤이라 그만큼은 '의심'으로만 알린다
      if (startCm > cfg.lineTolCm + 3) fouls.push({ code: 'line', sev: 'bad', label: '출발선 밟음(넘음)', detail: `발끝이 선을 약 ${round(startCm)}cm 넘었습니다.` });
      else if (startCm > cfg.lineTolCm) fouls.push({ code: 'line', sev: 'warn', label: '출발선 밟음 의심', detail: `발끝이 선을 약 ${round(startCm)}cm 넘어 보입니다. 앱이 잘못 봤을 수도 있으니 영상으로 확인하세요.` });
      else notes.push(`출발 발끝이 출발선 ${startCm >= 0 ? '위' : '뒤 ' + round(-startCm) + 'cm'}에 있었습니다. 기록은 발끝부터 잽니다.`);
      notes.push(`테이프 네 점으로 멀고 가까운 차이를 바로잡고, 관절 점과 신발 끝의 차이 ${cfg.shoeCm}cm를 뺐습니다.`);
    } else {
      if (standI) landX = median(standI.map(heelR));
      if (backWhy && backX < landX) { fouls.push({ code: 'back', sev: 'bad', label: `착지 후 ${backWhy}이 뒤에 닿음`, detail: `가장 뒤에 닿은 곳부터 재서 기록이 ${round((landX - backX) * slopeF / ppc)}cm 줄어듭니다.` }); landX = backX; }
      distCm = (landX - lineX) * slopeF / ppc;
      if (feetUse.length === 2 && skew > 2) notes.push(`두 발이 화면에서 ${round(skew)}cm 어긋나 보여 발마다 보정했습니다. 카메라를 정확히 옆에 두면 더 정확합니다.`);
      if (hasLine) {
        const over = (toeX - lineX) * slopeF / ppc;
        if (over > cfg.lineTolCm) fouls.push({ code: 'line', sev: 'bad', label: '출발선 밟음(넘음)', detail: `발끝이 선을 약 ${round(over)}cm 넘었습니다.` });
        else notes.push(`출발선까지 여유 ${round(Math.max(0, -over))}cm`);
      } else notes.push('출발선을 지정하지 않아 발끝부터 잰 거리입니다. 선 밟기 판정은 하지 않았습니다.');
      notes.push('카메라에서 먼 발과 가까운 발이 다르게 보이는 차이는 바로잡지 않았습니다. 정확히 재려면 "테이프 4점" 방식을 쓰세요.');
    }
    notes.unshift(standI ? '기록은 착지 후 똑바로 섰을 때 뒤꿈치 자리로 쟀습니다.' : '착지 후 똑바로 선 장면이 영상에 없어 착지 직후 발이 멈춘 자리로 쟀습니다. 일어설 때까지 찍어야 기준과 같아집니다.');
    // 이중 도약 / 준비 중 발 이동
    const w0 = idxAt(S, T[a] - 1.5);
    if (runs.some(r => r !== main && r.e < main.s && T[r.s] >= T[w0] && T[r.e] - T[r.s] >= 0.05)) fouls.push({ code: 'double', sev: 'warn', label: '두 번 뛴 것 같음', detail: '뛰기 직전에 발이 한 번 떴습니다. 영상을 확인하세요.' });
    else if (Math.abs(toeF(w0) - toeX) > 0.06 * H) fouls.push({ code: 'shift', sev: 'warn', label: '준비 중 발 이동 의심', detail: '뛰기 전 1.5초 안에 발 위치가 바뀌었습니다.' });

    // 자세 지표
    const sd = nearSide(S, [L.HIP, L.KNEE, L.ANK, L.SH]);
    const J = (i, key) => P[i][L[key][sd]], JK = [L.HIP[sd], L.KNEE[sd], L.ANK[sd]], JA = [L.HIP[sd], L.SH[sd], L.WR[sd]], JH = [L.SH[sd], L.HIP[sd], L.KNEE[sd]];
    const knee = i => ang(J(i, 'HIP'), J(i, 'KNEE'), J(i, 'ANK'));
    const hipA = i => ang(J(i, 'SH'), J(i, 'HIP'), J(i, 'KNEE'));
    const arm = i => { // 몸통 기준 팔 각도, 앞쪽(진행 방향)이 +
      const sh = J(i, 'SH'), hp = J(i, 'HIP'), wr = J(i, 'WR'), t = [hp[0] - sh[0], hp[1] - sh[1]], v = [wr[0] - sh[0], wr[1] - sh[1]];
      return -deg(Math.atan2(t[0] * v[1] - t[1] * v[0], t[0] * v[0] + t[1] * v[1])) * dir;
    };
    let deep = w0; for (let i = w0; i <= a; i++) if (knee(i) < knee(deep)) deep = i;
    let armBack = 0, ib = w0; for (let i = w0; i <= a; i++) if (-arm(i) > armBack) { armBack = -arm(i); ib = i; }
    let armTO = arm(main.s); if (armTO < -90) armTO += 360;
    const ext = Math.max(...[a - 1, a, main.s].map(knee)), hipExt = Math.max(...[a - 1, a, main.s].map(hipA));
    const com = i => { const sh = mid(P[i][11], P[i][12]), hp = mid(P[i][23], P[i][24]); return mid(sh, hp); };
    const Tf = T[b] - T[a], vx = Math.abs(com(b)[0] - com(a)[0]) / ppc / 100 / Tf;
    const vy = ((com(a)[1] - com(b)[1]) / ppc / 100 + 4.905 * Tf * Tf) / Tf;
    const leg = dist(J(b, 'HIP'), J(b, 'KNEE')) + dist(J(b, 'KNEE'), J(b, 'ANK'));
    const hipX = mid(P[j][23], P[j][24])[0] * dir, reach = (heelR(j) - hipX) / (median(P.map((f, i) => dist(J(i, 'HIP'), J(i, 'KNEE')) + dist(J(i, 'KNEE'), J(i, 'ANK')))) || leg) * 100;
    const le = idxAt(S, T[b] + 0.5); let landKnee = 180, il = b; for (let i = b; i <= le; i++) if (knee(i) < landKnee) { landKnee = knee(i); il = i; }
    let tuck = 180, it = main.s; for (let i = main.s; i <= main.e; i++) if (hipA(i) < tuck) { tuck = hipA(i); it = i; }
    // 공중·착지 세부: 무릎을 당긴 시점, 발이 뒤로 차올려졌는지, 발목 당김, 다리를 뻗은 시점, 닿는 순간 자세
    const shank = median(P.map((f, i) => dist(J(i, 'KNEE'), J(i, 'ANK')))) || 1, JF = [L.KNEE[sd], L.ANK[sd], L.TOE[sd]];
    let heelKick = -999, ik = main.s; for (let i = main.s; i <= main.e; i++) { const v = (J(i, 'KNEE')[1] - J(i, 'ANK')[1]) / shank * 100; if (v > heelKick) { heelKick = v; ik = i; } }
    const thighL = median(P.map((f, i) => dist(J(i, 'HIP'), J(i, 'KNEE')))) || 1; let ikf = -1; for (let i = main.s; i <= main.e; i++) if ((J(i, 'KNEE')[0] - J(i, 'HIP')[0]) * dir / thighL >= 0.5) { ikf = i; break; }
    const ankle = ang(J(main.e, 'KNEE'), J(main.e, 'ANK'), J(main.e, 'TOE'));
    let io = b; for (let i = b; i >= it; i--) { if (knee(i) >= 140) io = i; else if (io !== b || i < b - 1) break; }
    const legOut = knee(b) >= 140 || io !== b ? T[b] - T[io] : 0, tdKnee = Math.max(knee(b), knee(Math.max(main.s, b - 1))), tdHip = hipA(b);
    const R = cfg.ref;
    const metrics = [
      metric('knee', '앉는 깊이 (가장 깊이 앉았을 때 무릎 각도)', knee(deep), '°', R.knee, 0, T[deep], JK),
      metric('armBack', '팔을 뒤로 보낸 정도', armBack, '°', R.armBack, 0, T[ib], JA),
      metric('armTO', '뛰는 순간 팔 높이', armTO, '°', R.armTO, 0, T[main.s], JA),
      metric('ext', '뛰는 순간 무릎 펴짐', ext, '°', R.ext, 0, T[a], JK),
      metric('hipExt', '뛰는 순간 몸 펴짐', hipExt, '°', R.hipExt, 0, T[a], JH),
      metric('angle', '뛰어오르는 각도', deg(Math.atan2(vy, vx)), '°', R.angle, 0, T[main.s]),
      metric('tuck', '공중에서 무릎 당기기 (숫자가 작을수록 많이 당긴 것)', tuck, '°', R.tuck, 0, T[it], JH),
      metric('tuckT', '무릎을 앞으로 당긴 시점 (빠를수록 좋음)', ikf < 0 ? 100 : Math.max(0, (T[ikf] - T[a]) / Tf * 100), '%', R.tuckT, 0, T[ikf < 0 ? main.e : ikf], JH),
      metric('heelKick', '공중에서 발이 뒤로 들린 정도 (클수록 발바닥이 하늘을 봄)', heelKick, '%', R.heelKick, 0, T[ik], JK),
      metric('ankle', '착지 직전 발끝 당기기 (숫자가 작을수록 잘 당긴 것)', ankle, '°', R.ankle, 0, T[main.e], JF),
      metric('legOut', '다리를 뻗은 시점 (착지 몇 초 전)', legOut, '초', R.legOut, 2, T[io], JK),
      metric('tdKnee', '땅에 닿는 순간 무릎 펴짐', tdKnee, '°', R.tdKnee, 0, T[b], JK),
      metric('tdHip', '땅에 닿는 순간 상체를 숙인 정도 (숫자가 작을수록 많이 숙인 것)', tdHip, '°', R.tdHip, 0, T[b], JH),
      metric('reach', '착지할 때 발을 앞으로 내민 정도', reach, '%', R.reach, 0, T[j]),
      metric('landKnee', '착지하며 무릎을 굽힌 정도', landKnee, '°', R.landKnee, 0, T[il], JK),
      metric('speed', '뛰어나가는 속도', Math.hypot(vx, vy), 'm/s', R.speed, 2, T[main.s]),
      metric('flight', '공중에 떠 있던 시간', Tf, '초', null, 2)
    ];
    return {
      ok: true, event: 'jump', record: { value: round(distCm), unit: 'cm', valid: !fouls.some(f => f.sev === 'bad') }, fouls, notes, metrics,
      tips: tipsFor(metrics, JUMP_TIPS),
      events: [{ t: T[deep], label: '가장 깊이 앉은 순간' }, { t: T[main.s], label: '발이 떨어지는 순간' }, { t: T[j], label: '착지' }].concat(standI ? [{ t: T[standI[0]], label: '기록 자리' }] : []),
      anchor: T[main.s], geom: FM ? { base: -yF, dir, lineX: FM.at(startCm, yF), landX: FM.at(landCm, yF), toeX: FM.at(startCm, yF), k: 0, hasLine: true, floor: true } : { base, dir, lineX: lineX * dir, landX: landX * dir, toeX: toeX * dir, k, hasLine }, pxPerCm: ppc, hPx: H
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
    const J = (i, key) => P[i][L[key][sd]], JK = [L.HIP[sd], L.KNEE[sd], L.ANK[sd]], JA = [L.HIP[sd], L.SH[sd], L.WR[sd]], JH = [L.SH[sd], L.HIP[sd], L.KNEE[sd]];
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
      metric('validRate', '인정된 비율 (인정 개수 ÷ 시도 개수)', inTime.length ? valid.length / inTime.length * 100 : 0, '%', R.validRate),
      metric('paceDrop', '뒤로 갈수록 느려진 정도', paceDrop, '%', R.paceDrop),
      metric('kneeAngle', '무릎 각도', kneeA, '°', R.kneeAngle, 0, T[0], JK),
      metric('hipLiftN', '엉덩이 들림 횟수', liftN, '회', R.hipLiftN),
      metric('repTime', '1개 하는 데 걸린 시간 (평균)', durs.length ? mean(durs) : 0, '초', null, 2),
      metric('peakAngle', '올라왔을 때 상체 각도 (평균)', mean(inTime.map(r => r.peak)), '°', null, 0, inTime.length ? inTime[0].t : null, JH)
    ];
    const tips = [];
    const up = (reasons['팔꿈치가 무릎에 안 닿음'] || 0) + (reasons['상체 각도 부족'] || 0);
    if (up) tips.push({ key: 'up', label: `끝까지 안 올라온 횟수 ${up}회`, p: '상체를 끝까지 올리지 못해 무효가 됩니다.', fix: '턱을 당기고 가슴이 허벅지에 닿는다는 느낌으로 올라오세요. 먼저 정확히 닿는 것부터 익히고, 그다음에 속도를 올리세요.', ex: ['싯업 상단 2초 정지 3×12', 'V-업 3×10', '행잉 레그 레이즈 3×8', '장요근 밴드 니 드라이브 3×12'] });
    if (reasons['어깨가 바닥에 안 닿음']) tips.push({ key: 'down', label: `어깨가 매트에 안 닿음 ${reasons['어깨가 바닥에 안 닿음']}회`, p: '내려갈 때 어깨가 바닥에 닿기 전에 다시 올라옵니다.', fix: '등 윗부분이 매트를 찍는 감각을 확인하고 올라오세요. 메트로놈(분당 50~60회)에 맞춰 연습하면 리듬이 잡힙니다.', ex: ['템포 싯업(1초 업·1초 다운) 3×20', '할로우 바디 록 3×15'] });
    if (liftN) tips.push({ key: 'hip', label: `엉덩이 들림 ${liftN}회`, p: '몸을 튕겨서 올라오면서 엉덩이가 뜹니다.', fix: '엉덩이를 매트에 붙이고 복근으로만 말아 올리세요.', ex: ['데드버그 3×12', '할로우 바디 홀드 3×30초', '리버스 크런치 3×15'] });
    if (paceDrop > R.paceDrop[1]) tips.push({ key: 'pace', label: `뒤로 갈수록 ${round(paceDrop)}% 느려짐`, p: '후반으로 갈수록 속도가 크게 떨어집니다.', fix: '처음 20초를 전력의 90% 정도로 시작해서 끝까지 같은 빠르기로 하는 연습을 하세요.', ex: ['타바타 싯업(20초 수행·10초 휴식 ×8)', '1분 싯업 ×3세트(휴식 2분)', '플랭크 3×60초'] });
    if (kneeA > R.kneeAngle[1] || kneeA < R.kneeAngle[0]) tips.push({ key: 'knee', label: `무릎 각도 ${round(kneeA)}°`, p: '무릎 각도가 기준(약 90°)에서 벗어났습니다.', fix: '발뒤꿈치와 엉덩이 사이를 한 뼘 반 정도로 맞추고 시작하세요.', ex: [] });
    return {
      ok: true, event: 'situp', record: { value: valid.length, unit: '개', valid: true }, attempts: inTime.length, reasons, reps, buckets, metrics, tips,
      fouls: Object.keys(reasons).map(r => ({ code: r, sev: 'warn', label: `${r} ${reasons[r]}회`, detail: '무효 처리했습니다.' })),
      notes: [`시도 ${inTime.length}회 중 ${valid.length}회 인정`, `측정 시작 ${round(startT, 2)}초, 제한 ${cfg.limitSec}초`],
      events: reps.filter(r => r.reason !== '시작 전').map(r => ({ t: r.t, label: r.valid ? String(r.n) : '×', bad: !r.valid })), anchor: startT, series: { t: T, v: th.map(x => round(x)) }
    };
  }

  /* ───────── 공통: 설정 병합 ───────── */
  DEF.sargent = { ref: { knee: [70, 110], armTO: [120, 180], ext: [160, 180], drift: [0, 20], landKnee: [70, 130] } };
  DEF.flex = { holdSec: 2, kneeMin: 165, tipOffsetCm: 9, ref: { hipAngle: [0, 55], kneeAngle: [165, 180], sway: [0, 2] } };
  DEF.run10 = { calibMode: 'height', laps: 2, startMode: 'line', ref: { lean: [25, 60], turn: [0, 1.2], hipLow: [45, 75], finDrop: [-100, 8] } };
  DEF.run20 = JSON.parse(JSON.stringify(DEF.run10));
  DEF.shuttle = { startKmh: 8.5, missLimit: 2 };
  const mk = (id, o) => Object.assign({}, DEF[id], o.cfg, { ref: Object.assign({}, DEF[id].ref, o.cfg && o.cfg.ref) });

  /* ───────── 서전트 점프 ───────── */
  const SARGENT_TIPS = {
    'knee:high': JUMP_TIPS['knee:high'], 'knee:low': JUMP_TIPS['knee:low'], 'ext:low': JUMP_TIPS['ext:low'], 'landKnee:high': JUMP_TIPS['landKnee:high'],
    'armTO:low': { p: '뛰는 순간 팔이 덜 올라옵니다.', fix: '발이 떨어질 때 팔이 머리 위로 뻗어 있어야 합니다. 팔을 뒤에서 위로 한 번에 던지세요.', ex: ['팔 스윙 점프 3×6', '메디신볼 오버헤드 수직 던지기 3×8'] },
    'drift:high': { p: '뛴 자리에서 벗어나 앞이나 뒤로 떨어집니다. 위로 써야 할 힘이 옆으로 새는 것입니다.', fix: '발바닥 전체로 바닥을 똑바로 아래로 밀고, 뛴 자리에 그대로 착지하는 연습을 하세요.', ex: ['제자리 연속 점프(같은 자리 착지) 3×8', '박스 점프 3×5'] }
  };
  function analyzeSargent(frames, o) {
    o = o || {}; const cfg = mk('sargent', o), S = prep(frames);
    if (S.n < 12) return err('사람이 인식된 구간이 너무 짧습니다. 전신이 나오게 다시 촬영해 주세요.');
    const H = bodyH(S), ppc = o.pxPerCm || H / (o.heightCm || 170), T = S.t, P = S.p, n = S.n;
    const lowY = P.map(f => Math.max(f[29][1], f[30][1], f[31][1], f[32][1])), base = quant(lowY, 0.8), h = lowY.map(y => base - y), thr = 0.04 * H;
    const runs = []; let s = -1;
    for (let i = 0; i <= n; i++) { const air = i < n && h[i] > thr; if (air && s < 0) s = i; if (!air && s >= 0) { runs.push({ s, e: i - 1 }); s = -1; } }
    let main = null; runs.forEach(r => { const d = T[r.e] - T[r.s]; if (r.s > 1 && r.e < n - 2 && d >= 0.12 && d <= 1.3 && (!main || d > T[main.e] - T[main.s])) main = r; });
    if (!main) return err('점프 구간을 찾지 못했습니다. 준비 자세부터 착지 후 1초까지 발이 화면에 다 나오게 찍어 주세요.');
    // 체공 경계를 낮은 문턱으로 다시 잡는다: 처음 찾을 때 쓴 문턱(키의 4%)은 발이 꽤 뜬 뒤라 체공 시간이 짧게 잡힌다.
    // 프레임 평활 전 원래 값으로 잡아야 경계가 번지지 않는다.
    const rawP = new Map(frames.filter(f => f && f.p).map(f => [f.t, f.p]));
    const hR = T.map((t, i) => { const q = rawP.get(t); return q ? base - Math.max(q[29][1], q[30][1], q[31][1], q[32][1]) : h[i]; });
    const pre = hR.slice(Math.max(0, main.s - 45), Math.max(1, main.s - 8)), pm = median(pre), noise = 1.4826 * median(pre.map(v => Math.abs(v - pm))) || 0;
    const lo = Math.max(0.008 * H, 4 * noise);
    { let s0 = main.s, e0 = main.e; while (s0 > 1 && hR[s0 - 1] - pm > lo) s0--; while (e0 < n - 2 && hR[e0 + 1] - pm > lo) e0++; main = { s: s0, e: e0 }; }
    for (let i = 0; i < n; i++) h[i] = hR[i] - pm;
    const a = main.s - 1, b = main.e + 1;
    /* 체공 시간 (센서판과 같은 방식): 발이 바닥에서 떨어진 순간 ~ 다시 닿은 순간.
       프레임 사이에 떨어지고 닿으므로, 떨어진 직후 두 프레임의 발 속도로 0높이 순간을 거꾸로 짚고(착지도 같은 방식) 프레임보다 잘게 잡는다. */
    const dt0 = (T[n - 1] - T[0]) / (n - 1) || 1 / 30;
    const vUp = Math.max(1e-6, (h[Math.min(main.s + 1, main.e)] - h[main.s]) / ((T[Math.min(main.s + 1, main.e)] - T[main.s]) || dt0));
    const vDn = Math.max(1e-6, (h[Math.max(main.e - 1, main.s)] - h[main.e]) / ((T[main.e] - T[Math.max(main.e - 1, main.s)]) || dt0));
    const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
    const tOff = clamp(T[main.s] - Math.max(0, h[main.s]) / vUp, T[a], T[main.s]), tOn = clamp(T[main.e] + Math.max(0, h[main.e]) / vDn, T[main.e], T[b]);
    const Tf = tOn - tOff, flightH = 122.6 * Tf * Tf, Terr = dt0 / 4;
    const hipY = i => (P[i][23][1] + P[i][24][1]) / 2, hipX = i => (P[i][23][0] + P[i][24][0]) / 2;
    const sd = nearSide(S, [L.HIP, L.KNEE, L.ANK, L.SH]), J = (i, key) => P[i][L[key][sd]], JK = [L.HIP[sd], L.KNEE[sd], L.ANK[sd]], JA = [L.HIP[sd], L.SH[sd], L.WR[sd]];
    const knee = i => ang(J(i, 'HIP'), J(i, 'KNEE'), J(i, 'ANK'));
    const w0 = idxAt(S, T[a] - 1.5); let deep = w0; for (let i = w0; i <= a; i++) if (knee(i) < knee(deep)) deep = i;
    const st0 = idxAt(S, T[deep] - 1.2), stand = quant(P.slice(st0, deep + 1).map((f, i) => hipY(st0 + i)), 0.1);
    let top = main.s; for (let i = main.s; i <= main.e; i++) if (hipY(i) < hipY(top)) top = i;
    const rise = (stand - hipY(top)) / ppc, drift = Math.abs(hipX(b) - hipX(a)) / ppc;
    const dir = Math.sign(J(a, 'TOE')[0] - J(a, 'HEEL')[0]) || 1;
    const sh = J(main.s, 'SH'), hp = J(main.s, 'HIP'), wr = J(main.s, 'WR'), tv = [hp[0] - sh[0], hp[1] - sh[1]], av = [wr[0] - sh[0], wr[1] - sh[1]];
    let armTO = -deg(Math.atan2(tv[0] * av[1] - tv[1] * av[0], tv[0] * av[0] + tv[1] * av[1])) * dir; if (armTO < -90) armTO += 360;
    const ext = Math.max(...[a - 1, a, main.s].map(knee)), le = idxAt(S, T[b] + 0.5); let landKnee = 180, il = b; for (let i = b; i <= le; i++) if (knee(i) < landKnee) { landKnee = knee(i); il = i; }
    const fouls = [], footX = i => (P[i][29][0] + P[i][30][0]) / 2;
    // 무릎을 굽힌 채 내려오면 체공 시간이 늘어 기록이 실제보다 높게 나온다 (센서판도 같음)
    const tdKnee = Math.max(knee(b), knee(main.e)), hipDrop = (hipY(b) - hipY(a)) / H * 100;
    if (tdKnee < 150 || hipDrop > 4) fouls.push({ code: 'tuck', sev: 'warn', label: '무릎 굽힌 채 착지', detail: `닿는 순간 무릎 ${round(tdKnee)}°. 다리를 굽혀 내려오면 공중에 떠 있는 시간이 늘어 기록이 실제보다 높게 나옵니다. 발끝부터 다리를 편 채 착지하세요.` });
    if (Math.abs(footX(w0) - footX(a)) > 0.08 * H) fouls.push({ code: 'step', sev: 'warn', label: '뛰기 전에 발을 움직인 것 같음', detail: '점프 전 1.5초 안에 발 위치가 바뀌었습니다.' });
    if (drift > 30) fouls.push({ code: 'drift', sev: 'warn', label: '뛴 자리에서 벗어남', detail: `뛴 자리에서 약 ${round(drift)}cm 벗어나 착지했습니다.` });
    const R = cfg.ref, metrics = [
      metric('knee', '앉는 깊이 (가장 깊이 앉았을 때 무릎 각도)', knee(deep), '°', R.knee, 0, T[deep], JK), metric('armTO', '뛰는 순간 팔 높이', armTO, '°', R.armTO, 0, T[main.s], JA),
      metric('ext', '뛰는 순간 무릎 펴짐', ext, '°', R.ext, 0, T[a], JK), metric('drift', '뛴 자리에서 벗어난 거리', drift, 'cm', R.drift, 0, T[b]), metric('landKnee', '착지하며 무릎을 굽힌 정도', landKnee, '°', R.landKnee, 0, T[il], JK),
      metric('rise', '엉덩이가 올라간 높이 (참고용)', rise, 'cm', null, 1), metric('flight', '공중에 떠 있던 시간', Tf, '초', null, 3)];
    const fps = Math.round(1 / dt0), dH = 2 * 122.6 * Tf * Terr;
    return { ok: true, event: 'sargent', record: { value: round(flightH), unit: 'cm', valid: true }, fouls, metrics, tips: tipsFor(metrics, SARGENT_TIPS),
      notes: [`공중에 떠 있던 시간(${round(Tf, 3)}초)으로 높이를 계산했습니다. 점프 측정판과 같은 방식이라 키를 넣거나 기준선을 찍지 않아도 됩니다.`, `초당 ${fps}장으로 찍은 영상이라 높이가 ±${round(dH, 1)}cm 정도 틀릴 수 있습니다.${fps < 50 ? ' 60fps로 찍으면 틀리는 정도가 절반으로 줄어듭니다.' : ''}`],
      events: [{ t: T[deep], label: '가장 깊이 앉은 순간' }, { t: tOff, label: '발 떨어짐' }, { t: T[top], label: '최고점' }, { t: tOn, label: '발 닿음' }], anchor: tOff };
  }

  /* ───────── 유연성 (좌전굴) ───────── */
  function analyzeFlex(frames, o) {
    o = o || {}; const cfg = mk('flex', o), S = prep(frames);
    if (S.n < 10) return err('사람이 인식된 구간이 너무 짧습니다. 옆에서 앉은 자세 전체가 나오게 찍어 주세요.');
    const T = S.t, P = S.p, n = S.n, sd = nearSide(S, [L.HIP, L.KNEE, L.ANK]);
    const J = (i, key) => P[i][L[key][sd]], dir = Math.sign(median(P.map(f => f[L.ANK[sd]][0] - f[L.HIP[sd]][0]))) || 1, JK = [L.HIP[sd], L.KNEE[sd], L.ANK[sd]], JH = [L.SH[sd], L.HIP[sd], L.KNEE[sd]];
    // 몸 크기는 상체를 세우고 앉은 장면에서 잰다. 숙이면 어깨·엉덩이 점이 밀려 몸이 실제보다 크게 잡힌다.
    const leanOf = P.map(f => Math.abs(Math.atan2(f[L.SH[sd]][0] - f[L.HIP[sd]][0], f[L.HIP[sd]][1] - f[L.SH[sd]][1]))), lq = quant(leanOf, 0.3), up = []; for (let i = 0; i < n; i++) if (leanOf[i] <= lq) up.push(i);
    const H = up.length >= 5 ? bodyH({ n: up.length, t: up.map(i => T[i]), p: up.map(i => P[i]) }) : bodyH(S), ppc = H / (o.heightCm || 170);
    const tipX = smooth(P.map(f => { const ix = [19, 20].filter(i => f[i][2] > 0.3).map(i => f[i][0] * dir); return ix.length ? Math.max(...ix) : Math.max(f[15][0] * dir, f[16][0] * dir); }));
    // 가장 멀리 민 자리: 0.3초 이상 머문 가장 먼 지점 (한두 장 튄 값은 버린다). 측정판은 민 자리에 그대로 남으므로 이 값이 기록이다.
    let topPx = -1e9, ti = 0; for (let i = 0; i < n; i++) { const j = idxAt(S, T[i] + 0.3); if (T[j] - T[i] < 0.2) break; const v = quant(tipX.slice(i, j + 1), 0.2); if (v > topPx) { topPx = v; ti = i; } }
    if (topPx === -1e9) { topPx = Math.max(...tipX); ti = tipX.indexOf(topPx); }
    const tol = 2 * ppc, gapMax = 0.25; let bi = ti, bj = ti;
    for (let i = ti - 1, last = ti; i >= 0 && T[last] - T[i] <= gapMax; i--) if (tipX[i] >= topPx - tol) { bi = i; last = i; }
    for (let i = ti + 1, last = ti; i < n && T[i] - T[last] <= gapMax; i++) if (tipX[i] >= topPx - tol) { bj = i; last = i; }
    const heldSec = T[bj] - T[bi], held = heldSec >= cfg.holdSec - 0.15, win = []; for (let i = bi; i <= bj; i++) win.push(i);
    const sole = median(win.map(i => P[i][L.HEEL[sd]][0] * dir)), toCm = (x) => (x - sole) / ppc + cfg.tipOffsetCm, reach = tipX.map(toCm), best = toCm(topPx), peak = Math.max(...reach);
    const kneeMin = Math.min(...win.map(i => ang(J(i, 'HIP'), J(i, 'KNEE'), J(i, 'ANK')))), hipAngle = median(win.map(i => ang(J(i, 'SH'), J(i, 'HIP'), J(i, 'KNEE'))));
    const sway = quant(win.map(i => reach[i]), 0.9) - quant(win.map(i => reach[i]), 0.1), fouls = [];
    if (kneeMin < cfg.kneeMin) fouls.push({ code: 'knee', sev: 'bad', label: '무릎 굽힘', detail: `측정 중 무릎 각도가 ${round(kneeMin)}°까지 굽었습니다. (기준 ${cfg.kneeMin}° 이상)` });
    if (!held) fouls.push({ code: 'hold', sev: 'warn', label: '멈춰서 버틴 시간이 짧음', detail: `가장 멀리 민 자리에서 약 ${round(heldSec, 1)}초 머물렀습니다. 시험에서는 ${cfg.holdSec}초 이상 멈춰야 인정되는 경우가 많습니다. 기록은 가장 멀리 민 자리로 표시했습니다.` });
    const R = cfg.ref, metrics = [metric('hipAngle', '상체를 숙인 정도 (숫자가 작을수록 깊이 숙인 것)', hipAngle, '°', R.hipAngle, 0, T[bi], JH), metric('kneeAngle', '무릎 펴짐 (가장 많이 굽었을 때)', kneeMin, '°', R.kneeAngle, 0, T[bi], JK), metric('sway', '버티는 동안 흔들린 정도', sway, 'cm', R.sway, 1), metric('peak', '잠깐이라도 가장 멀리 닿은 거리', peak, 'cm', null, 1, T[reach.indexOf(peak)])];
    const tips = [];
    if (hipAngle > R.hipAngle[1]) tips.push({ p: '엉덩이 쪽은 그대로 두고 허리만 굽혀서 숙입니다.', fix: '등을 둥글게 말지 말고, 배꼽을 허벅지에 붙인다는 느낌으로 엉덩이 쪽부터 숙이세요. 숨을 내쉬며 천천히 내려갑니다.', ex: ['햄스트링 스트레칭(한 다리씩) 3×30초', '시티드 굿모닝 3×10', '고관절 90/90 스트레칭 3×30초', '폼롤러 종아리·햄스트링 각 1분'] });
    if (kneeMin < cfg.kneeMin) tips.push({ p: '무릎이 굽으면 무효입니다.', fix: '허벅지 앞쪽에 힘을 줘 무릎을 바닥으로 누른 채 숙이세요.', ex: ['앉아서 무릎 누르고 발끝 당기기 3×20초', '종아리 스트레칭 3×30초'] });
    if (!held) tips.push({ p: '끝까지 민 자리에서 오래 버티지 못합니다.', fix: '끝 지점에서 2초를 세고 손을 뗍니다. 튕기지 말고 천천히 미는 연습을 하세요.', ex: ['PNF 스트레칭(6초 버티고 더 숙이기) ×4'] });
    return { ok: true, event: 'flex', record: { value: round(best), unit: 'cm', valid: !fouls.some(f => f.sev === 'bad') }, fouls, metrics, tips,
      notes: ['발바닥이 닿는 면을 0cm로 보고 손끝이 넘어간 거리입니다. 손이 0.3초 이상 머문 가장 먼 자리를 기록으로 봅니다.', '손끝 위치는 앱이 추정하는 것이라 ±3cm 정도 틀릴 수 있습니다.', `앱이 손가락 마디까지만 찾기 때문에 손끝까지 ${cfg.tipOffsetCm}cm를 더했습니다. 설정에서 바꿀 수 있습니다.`],
      events: [{ t: T[bi], label: '유지 시작' }, { t: T[bj], label: '유지 끝' }], anchor: T[bi] };
  }

  /* ───────── 제자리멀리뛰기 표준 동작 (연구에 나온 권장 각도로 만든 관절 움직임. 실제 사람의 영상이 아니다) ───────── */
  function synthJump() {
    const Hpx = 420, fps = 30, GY = 900, X0 = 420, R = Math.PI / 180, G = 9.81 / 1.75;
    const L = { thigh: .245, shank: .245, trunk: .30, head: .13, uarm: .17, farm: .23 };
    // tr 상체 기울기, th 허벅지, sh 정강이(수직 아래 기준, 앞이 +), ft 발끝 내림, ar 팔(아래 기준, 앞이 +), el 팔꿈치
    const K = [
      { t: 0.00, tr: 3, th: 3, sh: -3, ft: 0, ar: 150, el: 10, a: 'toe' }, { t: 0.40, tr: 22, th: 30, sh: -20, ft: 0, ar: 30, el: 15, a: 'toe' },
      { t: 0.68, tr: 50, th: 60, sh: -32, ft: 0, ar: -115, el: 15, a: 'toe' }, { t: 0.92, tr: 32, th: -28, sh: -32, ft: 55, ar: 140, el: 10, a: 'toe' },
      { t: 1.10, tr: 25, th: 40, sh: -70, ft: 25, ar: 125, el: 10 }, { t: 1.28, tr: 40, th: 95, sh: -15, ft: 0, ar: 75, el: 10 }, { t: 1.42, tr: 55, th: 92, sh: 42, ft: -30, ar: 15, el: 5 },
      { t: 1.48, tr: 55, th: 86, sh: 40, ft: -32, ar: -30, el: 5, a: 'heel' }, { t: 1.72, tr: 55, th: 60, sh: -35, ft: 0, ar: 80, el: 15, a: 'heel' }, { t: 2.20, tr: 12, th: 10, sh: -8, ft: 0, ar: 20, el: 10, a: 'heel' }];
    const rot = (x, y, d) => [x * Math.cos(d * R) + y * Math.sin(d * R), -x * Math.sin(d * R) + y * Math.cos(d * R)];
    const pose = (k) => { // 엉덩이를 (0,0)에 둔 자세. y는 위가 +
      const kn = [L.thigh * Math.sin(k.th * R), -L.thigh * Math.cos(k.th * R)], an = [kn[0] + L.shank * Math.sin(k.sh * R), kn[1] - L.shank * Math.cos(k.sh * R)];
      const he = rot(-.05, -.04, k.ft), to = rot(.15, -.04, k.ft), sh = [L.trunk * Math.sin(k.tr * R), L.trunk * Math.cos(k.tr * R)];
      const el = [sh[0] + L.uarm * Math.sin(k.ar * R), sh[1] - L.uarm * Math.cos(k.ar * R)], wr = [el[0] + L.farm * Math.sin((k.ar + k.el) * R), el[1] - L.farm * Math.cos((k.ar + k.el) * R)];
      return { hip: [0, 0], kn, an, he: [an[0] + he[0], an[1] + he[1]], to: [an[0] + to[0], an[1] + to[1]], sh, el, wr, no: [sh[0] + L.head * (Math.sin(k.tr * .5 * R) + .25), sh[1] + L.head * Math.cos(k.tr * .5 * R)] };
    };
    const at = (t) => { let i = 0; while (i < K.length - 2 && t > K[i + 1].t) i++; const A = K[i], B = K[i + 1], s0 = Math.max(0, Math.min(1, (t - A.t) / (B.t - A.t))), s = s0 * s0 * (3 - 2 * s0), o = {}; ['tr', 'th', 'sh', 'ft', 'ar', 'el'].forEach(q => o[q] = A[q] + (B[q] - A[q]) * s); return o; };
    const TO = K[3], LD = K[7], pT = pose(TO), pL = pose(LD), hipT = [-pT.to[0], -pT.to[1]], Tf = LD.t - TO.t, fly = 0.88, hipL = [hipT[0] + fly, -pL.he[1]], D = hipL[0] + pL.he[0];
    const frames = [];
    for (let i = 0; i <= Math.round(K[K.length - 1].t * fps) + 15; i++) {
      const t = Math.min(i / fps, K[K.length - 1].t), k = at(t), P = pose(k); let hip;
      if (t <= TO.t) hip = [-P.to[0], Math.max(-P.to[1], -P.he[1])];
      else if (t < LD.t) { const s = (t - TO.t) / Tf; hip = [hipT[0] + fly * s, hipT[1] + (hipL[1] - hipT[1]) * s + .5 * G * Tf * Tf * s * (1 - s)]; }
      else hip = [D - P.he[0], Math.max(-P.he[1], -P.to[1])];
      const px = (q, dx) => [Math.round((X0 + (hip[0] + q[0]) * Hpx + (dx || 0)) * 10) / 10, Math.round((GY - (hip[1] + q[1]) * Hpx) * 10) / 10, 1], p = [];
      for (let j = 0; j < 33; j++) p.push(px(P.no));
      [[11, 'sh'], [13, 'el'], [15, 'wr'], [23, 'hip'], [25, 'kn'], [27, 'an'], [29, 'he'], [31, 'to']].forEach(([j, n]) => { p[j] = px(P[n], 0); p[j + 1] = px(P[n], 3); });
      frames.push({ t: i / fps + 1, p });
    }
    return { frames, takeoff: TO.t + 1, land: LD.t + 1, W: 1400, H: 1000, heightCm: 175, hPx: Hpx, distCm: Math.round(D * 175) };
  }

  /* ───────── 10m·20m 왕복달리기 ───────── */
  const RUN_LOSS = {
    start: { label: '출발해서 속도를 올리는 곳 (첫 구간 앞부분)', fix: '첫 3~4걸음은 머리부터 발목까지 일직선으로 앞으로 기울이고 바닥을 뒤로 밀어냅니다. 걸음을 일부러 크게 벌리지 말고 짧고 강하게 딛으세요.', ex: ['월 드릴(벽 밀고 무릎 올리기) 3×10', '푸시업 스타트 5회', '슬레드·밴드 저항 달리기 5×10m'] },
    reacc: { label: '턴하고 다시 속도를 올리는 곳 (턴 바로 뒤)', fix: '선을 밟은 발로 바닥을 강하게 밀어내고, 첫 두 걸음은 몸을 낮춘 채 짧고 빠르게 딛습니다. 턴하면서 상체가 먼저 서면 다시 빨라지는 데 시간이 걸립니다.', ex: ['5-10-5 프로 어질리티 6회', '래터럴 바운드 3×8', '스케이터 점프 3×10'] },
    mid: { label: '가운데 달리는 곳', fix: '10~20m 왕복은 거리가 짧아서 가운데에서도 계속 빨라지고 있어야 합니다. 팔을 크게 치고 무릎을 앞으로 끌어올려 걸음 수를 늘리세요.', ex: ['슬레드·밴드 저항 달리기 5×10m', '플라잉 10m 달리기 5회'] },
    decel: { label: '턴하려고 속도를 줄이는 곳 (턴 바로 앞)', fix: '너무 일찍 속도를 줄이고 있습니다. 선 두 걸음 전까지는 그대로 달리고, 마지막 두 걸음에서 엉덩이를 낮추며 짧게 멈추세요.', ex: ['감속 스텝 드릴(3보 감속 후 정지) 3×6', '사이드 런지 3×10', '낮은 자세 셔플 3×10m'] },
    finish: { label: '마지막 도착하는 곳', fix: '선을 지나 3m 뒤를 결승선이라고 생각하고 속도를 줄이지 말고 통과하세요.', ex: ['선 통과 후 5m 더 달리기 반복 6회'] }
  };
  const RUN_TIPS = {
    'lean:low': { p: '출발하자마자 상체가 섭니다. 앞으로 나가야 할 힘이 위로 빠집니다.', fix: '첫 3~4걸음은 머리부터 발목까지 일직선으로 앞으로 기울인 채 바닥을 뒤로 밀어내세요.', ex: ['월 드릴(벽 밀고 무릎 올리기) 3×10', '푸시업 스타트 5회', '슬레드·밴드 저항 달리기 5×10m'] },
    'lean:high': { p: '출발 때 상체가 너무 숙여져 첫걸음이 짧아집니다.', fix: '시선을 3m 앞 바닥에 두고, 넘어지지 않을 만큼만 기울이세요.', ex: ['폴링 스타트 5회', '3점 스타트 반복 5×5m'] },
    'turn:high': { p: '턴에 시간이 오래 걸립니다.', fix: '빨리 도는 사람은 선 바로 앞 걸음에서 무릎을 깊게 굽혀 몸을 낮추며 속도를 줄이고, 선을 밟는 발은 짧게 딛고 바로 밀어냅니다. 선 두 걸음 전부터 몸을 낮추고, 딛는 발끝과 몸을 돌아갈 방향으로 미리 틀어 두세요.', ex: ['5-10-5 프로 어질리티 6회', '감속 스텝 드릴(3보 감속 후 정지) 3×6', '래터럴 바운드 3×8'] },
    'hipLow:high': { p: '턴할 때 자세가 높습니다. 미끄러지거나 크게 돌게 됩니다.', fix: '선을 밟기 전 걸음에서 이미 낮아져 있어야 합니다. 무릎을 깊이 굽혀 엉덩이를 낮추고 상체를 돌아갈 쪽으로 기울이세요.', ex: ['사이드 런지 3×10', '스케이터 점프 3×10', '낮은 자세 셔플 3×10m'] },
    'finDrop:high': { p: '도착 전에 속도를 줄입니다.', fix: '선을 지나 3m 뒤를 결승선이라고 생각하고 그대로 통과하세요.', ex: ['선 통과 후 5m 더 달리기 반복 6회'] }
  };
  function analyzeRun(frames, o) {
    o = o || {}; const id = o.dist === 20 ? 'run20' : 'run10', cfg = mk(id, o), D = (o.dist || 10) * 100;
    const F = frames.filter(f => f && f.p);
    if (F.length < 10) return err('영상에서 사람을 거의 찾지 못했습니다. 폰을 가로로 눕혀 머리부터 발끝까지 나오게 찍고, 사람이 너무 작게 찍히지 않도록 달리는 줄에서 5~8m 거리에서 찍어 주세요. 어둡거나 역광이어도 잘 못 찾습니다.');
    if (!isFinite(o.lineX)) { // 간편 측정: 출발 자세의 앞발 끝을 출발선으로 본다
      if (!o.auto) return err('출발선 위치를 먼저 찍어 주세요.');
      const hx0 = F.map(f => (f.p[23][0] + f.p[24][0]) / 2), b0 = median(hx0.slice(0, 5)), H0 = bodyH({ p: F.map(f => f.p) }), mv = hx0.findIndex(x => Math.abs(x - b0) > 0.3 * H0);
      if (mv < 0) return err('달리기 시작을 찾지 못했습니다. 출발 전 서 있는 자세부터 찍어 주세요.');
      const d0 = Math.sign(hx0[mv] - b0), still = F.filter((f, i) => i < mv && Math.abs(hx0[i] - b0) < 0.05 * H0).slice(0, 15);
      o = Object.assign({}, o, { lineX: d0 * median(still.map(f => Math.max(...[29, 30, 31, 32].map(k => f.p[k][0] * d0)))), autoLine: true });
    }
    const H = bodyH({ p: F.map(f => f.p) }), far = isFinite(o.farX), ppc = far ? Math.abs(o.farX - o.lineX) / D : H / (o.heightCm || 170);
    const T = F.map(f => f.t), hx = F.map(f => (f.p[23][0] + f.p[24][0]) / 2), n = F.length, x0 = median(hx.slice(0, 5));
    let d = far ? Math.sign(o.farX - o.lineX) : -Math.sign(x0 - o.lineX); if (!d) d = Math.sign(hx[Math.min(n - 1, 15)] - x0) || 1;
    const u = hx.map(x => (x - o.lineX) * d / ppc), dt = median(T.slice(1).map((t, i) => t - T[i]));
    const cross = (i, c) => T[i - 1] + (c - u[i - 1]) / ((u[i] - u[i - 1]) || 1e-9) * (T[i] - T[i - 1]), near = i => T[i] - T[i - 1] < 0.3;
    let si = -1; for (let i = 1; i < n; i++) if (near(i) && u[i - 1] < 0 && u[i] >= 0) { si = i; break; }
    if (si < 0) return err('출발선을 넘는 장면을 찾지 못했습니다. 출발 전 자세부터 찍고, 출발선 위치를 다시 찍어 주세요.');
    const tLine = cross(si, 0); let mi = 0; for (let i = 0; i < si; i++) if (Math.abs(hx[i] - x0) > 0.08 * H) { mi = i; break; }
    const tMove = mi ? T[mi - 1] : T[Math.max(0, si - 1)], tStart = cfg.startMode === 'move' ? tMove : tLine;
    const us = u.slice(si), delta = Math.min(200, 0.6 * Math.max(...us)), his = pivots(us, delta).filter(q => q.type === 'hi').map(q => q.i + si).slice(0, cfg.laps);
    if (!his.length) return err('반대편으로 갔다가 돌아오는 장면을 찾지 못했습니다. 도착할 때까지 끊지 않고 찍어 주세요.');
    const fouls = [];
    if (his.length < cfg.laps) fouls.push({ code: 'laps', sev: 'warn', label: `왕복 ${cfg.laps}회 중 ${his.length}회만 확인`, detail: '왕복 횟수 설정이나 촬영 구간을 확인하세요.' });
    const last = his[his.length - 1]; let fi = -1; for (let i = last + 1; i < n; i++) if (near(i) && u[i - 1] >= 0 && u[i] < 0) { fi = i; break; }
    if (fi < 0) return err('출발선으로 돌아와 통과하는 장면을 찾지 못했습니다. 선을 지나칠 때까지 찍어 주세요.');
    const tFin = cross(fi, 0), fu = (i, fn) => fn(...[29, 30, 31, 32].map(k => (F[i].p[k][0] - o.lineX) * d / ppc));
    const sdn = nearSide({ p: F.map(f => f.p) }, [L.HIP, L.KNEE, L.ANK]), leg = median(F.map(f => dist(f.p[L.HIP[sdn]], f.p[L.KNEE[sdn]]) + dist(f.p[L.KNEE[sdn]], f.p[L.ANK[sdn]])));
    const hipPct = i => (Math.max(...[29, 30, 31, 32].map(k => F[i].p[k][1])) - (F[i].p[23][1] + F[i].p[24][1]) / 2) / leg * 100;
    const turnDur = [], hipLows = [], marks = [tStart], events = [{ t: tStart, label: '출발' }];
    const zone = (c, inside) => { let a = c, b = c; while (a > 0 && near(a) && inside(u[a - 1])) a--; while (b < n - 1 && near(b + 1) && inside(u[b + 1])) b++; turnDur.push(T[b] - T[a]); let m = 200; for (let i = a; i <= b; i++) m = Math.min(m, hipPct(i)); hipLows.push(m); };
    his.forEach((hi, k) => {
      if (far) { let fm = -1e9; for (let i = Math.max(0, hi - 3); i <= Math.min(n - 1, hi + 3); i++) fm = Math.max(fm, fu(i, Math.max)); if (fm < D - 5) fouls.push({ code: 'far' + k, sev: 'warn', label: `${k + 1}번째 반대편 턴에서 선에 못 미침`, detail: `선까지 약 ${round(D - fm)}cm 남았습니다.` }); zone(hi, v => v > D - 150); events.push({ t: T[hi], label: '턴' }); }
      if (k < his.length - 1) { let li = hi; for (let i = hi; i <= his[k + 1]; i++) if (u[i] < u[li]) li = i; let fm = 1e9; for (let i = Math.max(0, li - 3); i <= Math.min(n - 1, li + 3); i++) fm = Math.min(fm, fu(i, Math.min)); if (fm > 5) fouls.push({ code: 'near' + k, sev: 'warn', label: `${k + 1}번째 출발선 턴에서 선에 못 미침`, detail: `선까지 약 ${round(fm)}cm 남았습니다.` }); zone(li, v => v < 150); marks.push(T[li]); events.push({ t: T[li], label: '턴' }); }
    });
    marks.push(tFin); events.push({ t: tFin, label: '도착' });
    const leanF = F.filter(f => f.t >= tLine + 0.05 && f.t <= tLine + 0.6).map(f => { const sh = mid(f.p[11], f.p[12]), hp = mid(f.p[23], f.p[24]); return deg(Math.atan2((sh[0] - hp[0]) * d, hp[1] - sh[1])); });
    const R = cfg.ref, metrics = [];
    if (leanF.length) metrics.push(metric('lean', '출발 직후 상체를 앞으로 기울인 정도', mean(leanF), '°', R.lean, 0, tLine + 0.3, [12, 24, 26]));
    if (turnDur.length) { metrics.push(metric('turn', '턴에 걸린 시간 (선 앞 1.5m에 들어와서 나갈 때까지)', mean(turnDur), '초', R.turn, 2, events.length > 1 ? events[1].t : null)); metrics.push(metric('hipLow', '턴할 때 자세 높이 (낮을수록 좋음)', mean(hipLows), '%', R.hipLow, 0, events.length > 1 ? events[1].t : null)); }
    const back = c => { for (let i = fi; i > last; i--) if (near(i) && u[i - 1] >= c && u[i] < c) return cross(i, c); return null; }, t150 = back(150), t300 = back(300);
    if (t150 != null && t300 != null && t150 > t300) metrics.push(metric('finDrop', '도착 직전에 속도가 줄어든 정도 (마지막 1.5m)', (1 - (t150 - t300) / (tFin - t150)) * 100, '%', R.finDrop));
    for (let i = si; i < n; i++) if (near(i) && u[i - 1] < 200 && u[i] >= 200) { metrics.push(metric('first2m', '출발선에서 첫 2m까지', cross(i, 200) - tLine, '초', null, 2)); break; }
    metrics.push(metric('react', '첫 움직임부터 출발선 통과까지', Math.max(0, tLine - tMove), '초', null, 2));
    marks.slice(1).forEach((t, k) => metrics.push(metric('split' + k, `${k + 1}번째 왕복`, t - marks[k], '초', null, 2)));
    // ── 구간별 속도: 출발선↔반대편 한 번을 한 구간으로 보고, 구간마다 앞·가운데·뒤 1/3로 나눠 어디서 시간을 잃는지 본다
    const bnd = [{ t: tLine, u: 0, i: si }]; his.forEach((hi, k) => { bnd.push({ t: T[hi], u: u[hi], i: hi }); if (k < his.length - 1) { let li = hi; for (let i = hi; i <= his[k + 1]; i++) if (u[i] < u[li]) li = i; bnd.push({ t: T[li], u: u[li], i: li }); } }); bnd.push({ t: tFin, u: 0, i: fi });
    const sections = []; for (let k = 0; k < bnd.length - 1; k++) {
      const A = bnd[k], B = bnd[k + 1], sg = Math.sign(B.u - A.u) || 1, cm = Math.abs(B.u - A.u), sec = B.t - A.t; if (cm < 200 || sec <= 0) continue;
      const at = (c) => { for (let i = Math.max(1, A.i); i <= B.i; i++) if (near(i) && (u[i] - c) * sg >= 0 && (u[i - 1] - c) * sg < 0) return cross(i, c); return null; };
      const t1 = at(A.u + (B.u - A.u) / 3), t2 = at(A.u + (B.u - A.u) * 2 / 3), ok = t1 != null && t2 != null && t1 > A.t && t2 > t1 && B.t > t2, sp = (d0, d1) => round(cm / 3 / 100 / (d1 - d0), 2);
      sections.push({ n: sections.length + 1, out: sg > 0, t0: round(A.t, 2), t1: round(B.t, 2), sec: round(sec, 2), cm: round(cm), mps: round(cm / 100 / sec, 2), ph: ok ? [sp(A.t, t1), sp(t1, t2), sp(t2, B.t)] : null, pt: ok ? [t1 - A.t, t2 - t1, B.t - t2] : null });
    }
    const loss = []; const full = sections.filter(s => s.ph);
    if (full.length >= 2) {
      const vmax = Math.max(...full.map(s => Math.max(...s.ph))), L = { start: 0, reacc: 0, mid: 0, decel: 0, finish: 0 }, lastN = sections[sections.length - 1].n;
      full.forEach(s => { const ideal = s.cm / 3 / 100 / vmax, d = s.pt.map(x => Math.max(0, x - ideal)); L[s.n === 1 ? 'start' : 'reacc'] += d[0]; L.mid += d[1]; L[s.n === lastN ? 'finish' : 'decel'] += d[2]; });
      const sl = sections.slice().sort((p, q) => p.mps - q.mps)[0], fa = sections.slice().sort((p, q) => q.mps - p.mps)[0];
      Object.keys(L).forEach(k => { if (L[k] > 0.03) loss.push(Object.assign({ type: k, sec: round(L[k], 2) }, RUN_LOSS[k])); }); loss.sort((p, q) => q.sec - p.sec);
      if (sl.n !== fa.n && (fa.mps - sl.mps) / fa.mps > 0.06) loss.slow = { n: sl.n, mps: sl.mps, best: fa.n, bestMps: fa.mps, late: sl.n > sections.length / 2 };
      loss.vmax = round(vmax, 2);
    }
    return { ok: true, event: id, record: { value: round(tFin - tStart, 2), unit: '초', valid: !fouls.some(f => f.sev === 'bad') }, fouls, metrics, tips: tipsFor(metrics, RUN_TIPS), events, anchor: tStart, sections, loss, lossSlow: loss.slow || null, vmax: loss.vmax || null,
      notes: [cfg.startMode === 'move' ? '첫 움직임부터 출발선 복귀까지 잰 시간입니다.' : '몸통이 출발선을 넘은 순간부터 다시 넘어 돌아온 순간까지입니다.', `영상으로 재는 것이라 시간이 ±${round(dt / 2, 2)}초 정도 틀릴 수 있습니다.`, far ? '양쪽 선 사이 거리를 기준으로 거리를 쟀습니다.' : '키를 기준으로 거리를 어림했기 때문에, 턴할 때 선에 닿았는지는 정확하지 않을 수 있습니다.'].concat(o.autoLine ? ['출발선을 찍지 않아 출발 자세의 앞발 끝을 출발선으로 봤습니다.'] : []),
      vlines: [{ x: o.lineX, label: '출발선' }].concat(far ? [{ x: o.farX, label: '반대편' }] : []) };
  }

  const BASIS = {
    jump: [['제자리멀리뛰기 최적 도약 각도 (Wakai & Linthorne, 2005)', 'https://bura.brunel.ac.uk/bitstream/2438/537/1/%27Standing%20Long%20Jump%27%20Post-print.pdf'], ['제자리멀리뛰기의 팔 동작과 무릎 각도 (sponet)', 'https://sponet.de/sponet/Record/4011476?lng=en']],
    sargent: [['반동 깊이와 점프 높이 (PMC11208864)', 'https://pmc.ncbi.nlm.nih.gov/articles/PMC11208864']],
    run: [['180° 방향 전환을 빠르게 하는 요인 (Dos\'Santos 외, 2020)', 'https://e-space.mmu.ac.uk/626014'], ['세계 정상급 단거리 선수의 스타트 (Walker 외, 2021)', 'https://researchportal.bath.ac.uk/en/publications/kinematic-factors-associated-with-start-performance-in-world-clas/']]
  };
  const api = { BASIS, DEF, prep, bodyH, floorModel, analyzeJump, analyzeSitup, analyzeSargent, analyzeFlex, analyzeRun, synthJump, pivots, ang };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Analysis = api;
})(typeof self !== 'undefined' ? self : this);
