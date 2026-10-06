(() => {
  "use strict";

  const $ = (s, el = document) => el.querySelector(s);
  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(d));

  const S = {
    joints: [], sel: 1, win: 10, control: false, urls: [], sim: false,
    status: null, trials: [], review: null, dismissed: false, activity: null,
    wakeLock: null, liveKeep: 16,
  };
  const B = { arm: null, imu: null };

  function resetBuffers() {
    const J = S.joints.length;
    const cols = () => Array.from({ length: J }, () => []);
    B.arm = { t: [], pos: cols(), cmd: cols(), tgt: cols(), vel: cols(), load: cols() };
    B.imu = { t: [], acc: [[], [], []], gyro: [[], [], []] };
  }

  // ------------------------------------------------------------ charts
  const charts = {};
  let xMax = 0;

  function jointColors() { return S.joints.map((_, i) => css(`--j${i % 6}`)); }

  function mkChart(id, series, { y2 = false } = {}) {
    const el = document.getElementById(id);
    el.innerHTML = "";
    const axis = css("--muted"), grid = css("--grid");
    const axes = [
      { stroke: axis, grid: { stroke: grid, width: 1 }, ticks: { stroke: grid, width: 1 }, size: 26,
        values: (u, vs) => vs.map((v) => (S.review ? `${v.toFixed(0)}s` : `${(v - xMax).toFixed(0)}s`)) },
      { stroke: axis, grid: { stroke: grid, width: 1 }, ticks: { show: false }, size: 48 },
    ];
    const scales = { x: { time: false } };
    if (y2) {
      axes.push({ scale: "y2", side: 1, stroke: axis, grid: { show: false }, ticks: { show: false }, size: 44 });
      scales.y2 = {};
    }
    const opts = {
      width: Math.max(200, el.clientWidth), height: Number(el.dataset.h || 160),
      padding: [6, y2 ? 4 : 10, 0, 0], scales, axes,
      legend: { live: false },
      cursor: { drag: { x: true, y: false, setScale: true }, points: { show: false } },
      series: [{}, ...series.map((s) => ({
        label: s.label, stroke: s.color, width: s.width || 1.4, dash: s.dash,
        scale: s.scale || "y", points: { show: false }, spanGaps: false,
      }))],
    };
    const u = new uPlot(opts, [[], ...series.map(() => [])], el);
    new ResizeObserver(() => {
      const w = Math.max(200, el.clientWidth);
      if (w !== u.width) u.setSize({ width: w, height: u.height });
    }).observe(el);
    return u;
  }

  function buildCharts() {
    const jc = jointColors();
    const j = S.sel, jn = S.joints[j] || "";
    $("#jname").textContent = jn;
    $("#jname2").textContent = jn;
    charts.pos = mkChart("c-pos", S.joints.map((n, i) => ({ label: n, color: jc[i] })));
    charts.joint = mkChart("c-joint", [
      { label: "명령 (Goal)", color: css("--j1"), width: 1.4 },
      { label: "실제", color: css("--j0"), width: 1.8 },
    ]);
    charts.eff = mkChart("c-eff", [
      { label: "속도 °/s", color: css("--j0") },
      { label: "부하 % (PWM)", color: css("--j3"), scale: "y2" },
    ], { y2: true });
    const xyz = [css("--x"), css("--y"), css("--z")];
    charts.gyro = mkChart("c-gyro", ["x", "y", "z"].map((a, i) => ({ label: a, color: xyz[i] })));
    charts.acc = mkChart("c-acc", ["x", "y", "z"].map((a, i) => ({ label: a, color: xyz[i] })));
    requestRender(true);
  }

  function setChart(u, data, xr) {
    u.setData(data, false);
    u.setScale("x", xr);
  }

  let lastRender = 0, pending = false, force = false;
  function requestRender(f) {
    force = force || !!f;
    if (pending) return;
    pending = true;
    requestAnimationFrame(renderFrame);
  }
  function renderFrame(ts) {
    pending = false;
    if (!force && ts - lastRender < 50) { requestRender(); return; }
    force = false;
    lastRender = ts;
    if (S.review) renderReview(); else renderLive();
  }

  function renderLive() {
    if (!charts.pos || !B.arm) return;
    const a = B.arm, m = B.imu, j = S.sel;
    const tEnd = Math.max(a.t.length ? a.t[a.t.length - 1] : 0, m.t.length ? m.t[m.t.length - 1] : 0);
    if (!tEnd) return;
    xMax = tEnd;
    const xr = { min: tEnd - S.win, max: tEnd };
    setChart(charts.pos, [a.t, ...a.pos], xr);
    setChart(charts.joint, [a.t, a.cmd[j], a.pos[j]], xr);
    setChart(charts.eff, [a.t, a.vel[j], a.load[j]], xr);
    setChart(charts.gyro, [m.t, ...m.gyro], xr);
    setChart(charts.acc, [m.t, ...m.acc], xr);
  }

  function renderReview() {
    const d = S.review && S.review.data;
    if (!d || !charts.pos) return;
    const j = S.sel;
    const xr = { min: d.t[0], max: d.t[d.t.length - 1] };
    setChart(charts.pos, [d.t, ...d.pos], xr);
    setChart(charts.joint, [d.t, d.cmd[j], d.pos[j]], xr);
    setChart(charts.eff, [d.t, d.vel[j], d.load[j]], xr);
    setChart(charts.gyro, [d.t, ...d.gyro], xr);
    setChart(charts.acc, [d.t, ...d.acc], xr);
  }

  // ------------------------------------------------------------ live data
  function trim(buf, keepFrom) {
    let i = 0;
    while (i < buf.t.length && buf.t[i] < keepFrom) i++;
    if (!i) return;
    buf.t.splice(0, i);
    for (const k of Object.keys(buf)) {
      if (k === "t") continue;
      for (const col of buf[k]) col.splice(0, i);
    }
  }

  function onData(msg) {
    if (!B.arm) return;
    if (msg.reset) resetBuffers();
    const J = S.joints.length;
    if (msg.arm) {
      const a = msg.arm;
      for (let n = 0; n < a.t.length; n++) {
        B.arm.t.push(a.t[n]);
        for (let j = 0; j < J; j++) {
          B.arm.pos[j].push(a.pos[n][j]);
          B.arm.cmd[j].push(a.cmd[n][j]);
          B.arm.tgt[j].push(a.tgt[n][j]);
          B.arm.vel[j].push(a.vel[n][j]);
          B.arm.load[j].push(a.load[n][j]);
        }
      }
    }
    if (msg.imu) {
      const m = msg.imu;
      for (let n = 0; n < m.t.length; n++) {
        B.imu.t.push(m.t[n]);
        for (let k = 0; k < 3; k++) { B.imu.acc[k].push(m.acc[n][k]); B.imu.gyro[k].push(m.gyro[n][k]); }
      }
    }
    const tEnd = Math.max(B.arm.t.at(-1) || 0, B.imu.t.at(-1) || 0);
    trim(B.arm, tEnd - S.liveKeep);
    trim(B.imu, tEnd - S.liveKeep);
    if (!S.review) requestRender();
  }

  // ------------------------------------------------------------ status UI
  function pill(id, cls, text, title) {
    const el = document.getElementById(id);
    el.className = `pill ${cls}`;
    el.querySelector("b").textContent = text;
    el.title = title || "";
  }

  function renderStatus(st) {
    S.status = st;
    const arm = st.arm || {}, cf = st.cf || {}, clk = st.clock || {};
    const f = arm.follower || {}, l = arm.leader || {};
    const notes = Object.entries(arm.port_notes || {}).map(([p, n]) => `${p}: ${n}`).join("\n");

    if (f.connected) {
      const miss = (f.missing || []).length ? ` · ID ${f.missing.join(",")} 없음` : "";
      pill("p-follower", arm.rate_hz > 0 ? (miss ? "warn" : "ok") : "warn",
        `${fmt(arm.rate_hz, 0)} Hz · ${fmt(f.voltage, 1)} V${miss}`, `${f.port}\n${arm.message || ""}`);
    } else {
      pill("p-follower", "off", "연결 안 됨", `${arm.message || ""}\n${notes}`);
    }
    if (cf.state === "connected" && cf.rate_hz > 0) {
      const bat = cf.battery_v ? ` · ${fmt(cf.battery_v, 2)} V` : "";
      pill("p-cf", cf.battery_v && cf.battery_v < 3.5 ? "warn" : "ok", `${fmt(cf.rate_hz, 0)} Hz${bat}`,
        `${cf.uri}${cf.link_quality != null ? `\n링크 품질 ${fmt(cf.link_quality, 0)}%` : ""}`);
    } else if (cf.state === "disabled") {
      pill("p-cf", "off", "사용 안 함", "");
    } else {
      pill("p-cf", cf.state === "connecting" ? "warn" : "off", cf.state === "connecting" ? "연결 중" : "연결 안 됨",
        `${cf.uri || ""}\n${cf.message || ""}`);
    }
    if (clk.ready) {
      const j = clk.envelope_jitter_ms;
      pill("p-sync", j < 1 ? "ok" : "warn", `±${fmt(j, 2)} ms · ${fmt(clk.drift_ppm, 0)} ppm`,
        "Crazyflie 시계 → 이 Mac 시계 매핑 (최근 30초 하한선 피팅)\n±: 피팅 잔차, ppm: 두 시계 속도 차");
    } else {
      pill("p-sync", "off", "IMU 대기", "IMU 데이터가 몇 초 쌓이면 동기화 추정이 시작됩니다");
    }

    // recording
    const rec = st.rec || {};
    const btn = $("#btn-rec");
    btn.classList.toggle("on", !!rec.recording);
    btn.textContent = rec.recording ? "■ 기록 정지" : "● 기록 시작";
    const info = $("#rec-info");
    info.classList.toggle("on", !!rec.recording);
    info.textContent = rec.recording
      ? `${rec.id.split("_")[0]} · ${fmtDur(rec.elapsed_s)}${rec.auto ? " · 자동" : ""}`
      : (st.moving ? "움직임 감지" : "");
    const auto = $("#auto");
    if (document.activeElement !== auto) auto.checked = !!st.auto_trial;
    document.title = rec.recording ? `● ${rec.id.split("_")[0]} — NA Logger` : "NA Logger";

    renderJointTable(st.joints);
    renderAlign(st.align || {}, st.calib, arm);
    renderStandby(st);
    updateRecordingTrial(rec);
  }

  function fmtDur(s) {
    if (s == null) return "";
    const m = Math.floor(s / 60), r = s - m * 60;
    return m ? `${m}:${r.toFixed(1).padStart(4, "0")}` : `${r.toFixed(1)} s`;
  }

  function renderJointTable(js) {
    const tb = $("#jtable tbody");
    const jc = jointColors();
    if (tb.children.length !== S.joints.length) {
      tb.innerHTML = S.joints.map((n, i) =>
        `<tr data-j="${i}"><td><span class="sw" style="background:${jc[i]}"></span>${esc(n)}</td>${"<td></td>".repeat(8)}</tr>`).join("");
      tb.querySelectorAll("tr").forEach((tr) => tr.addEventListener("click", () => selectJoint(+tr.dataset.j)));
    }
    [...tb.children].forEach((tr, i) => {
      tr.classList.toggle("sel", i === S.sel);
      const c = tr.children;
      if (!js || js.age_s > 2) {
        for (let k = 1; k < 9; k++) c[k].textContent = "—";
        tr.classList.remove("bad");
        return;
      }
      tr.classList.toggle("bad", !js.ok[i]);
      const err = js.cmd[i] != null && js.pos[i] != null ? js.cmd[i] - js.pos[i] : null;
      c[1].textContent = fmt(js.pos[i], 1);
      c[2].textContent = fmt(js.cmd[i], 1);
      c[3].textContent = fmt(err, 1);
      c[4].textContent = fmt(js.vel[i], 0);
      c[5].textContent = fmt(js.load[i], 1);
      c[6].textContent = fmt(js.cur[i], 0);
      const t = js.temp[i];
      c[7].textContent = t == null ? "—" : t;
      c[7].className = t >= 65 ? "vhot" : t >= 55 ? "hot" : "";
      c[8].textContent = fmt(js.volt[i], 1);
    });
  }

  // ------------------------------------------------------------ alignid
  const ALIGN_TXT = { idle: ["대기", ""], checking: ["점검 중", "warn"], running: ["동작 중", "rec"],
    solving: ["계산 중", "warn"], done: ["완료", "ok"], error: ["실패", "warn"], aborted: ["중단됨", "warn"] };
  let alignCheck = null;

  function renderAlign(al, calib, arm) {
    const [txt, cls] = ALIGN_TXT[al.state] || ["—", ""];
    const badge = $("#align-badge");
    badge.textContent = al.state === "idle" && calib ? `적용 중: ${calib.quality === "good" ? "양호" : "확인 필요"}` : txt;
    badge.className = `badge ${al.state === "idle" && calib ? (calib.quality === "good" ? "ok" : "warn") : cls}`;
    const busy = ["checking", "running", "solving"].includes(al.state);
    $("#align-progress").hidden = !busy && al.state !== "done";
    $("#align-bar").style.width = `${Math.round((al.progress || 0) * 100)}%`;
    let step = busy ? `${al.step || ""}` : (al.message || "");
    if (alignCheck && !busy) step = alignCheck;
    $("#align-step").innerHTML = esc(step) + ((al.problems || []).length && !busy && al.state !== "done"
      ? `<ul class="problems">${al.problems.map((p) => `<li>${esc(p)}</li>`).join("")}</ul>` : "");
    const ctl = $("#align-ctl");
    const torque = ((S.status || {}).joints || {}).torque || [];
    const torqueOn = torque.some((v) => v === 1);
    const key = `${S.control}|${busy}|${torqueOn}`;
    if (ctl.dataset.key !== key) {
      ctl.dataset.key = key;
      if (!S.control) {
        ctl.innerHTML = `<button class="btn" data-a="check">경로 점검</button>
          <span class="muted small">정렬 시작은 로거 Mac에서만 가능합니다.</span>`;
      } else if (busy) {
        ctl.innerHTML = `<button class="btn danger" data-a="abort">중단 (현재 자세 유지)</button>`;
      } else {
        ctl.innerHTML = `<button class="btn" data-a="check">경로 점검</button>
          <button class="btn primary" data-a="start">정렬 시작 (약 1.5분)</button>
          <button class="btn primary" data-a="start_precise" title="정지 자세 24개로 가속도계 축별 스케일까지 보정">정밀 정렬 (약 3분)</button>
          <button class="btn danger" data-a="torque_off" ${torqueOn ? "" : "disabled"} title="팔로워 토크 끄기 (팔을 손으로 옮길 때)">토크 끄기</button>`;
      }
      ctl.querySelectorAll("button").forEach((b) => b.addEventListener("click", () => alignAction(b.dataset.a)));
    }
    renderCalib(calib);
  }

  async function alignAction(a) {
    if (a === "torque_off") {
      if (!confirm("팔로워 토크를 끄면 팔이 중력으로 떨어질 수 있습니다. 팔을 받친 상태인가요?")) return;
      const r = await fetch("/api/teleop/torque_off", { method: "POST" });
      if (!r.ok) toast(await r.text());
      return;
    }
    const precise = a === "start_precise";
    if (precise) a = "start";
    if (a === "start" && !confirm(`팔이 약 ${precise ? "3분" : "90초"} 동안 자동으로 움직입니다.\n주변을 비우고, 드론이 켜져 있는지 확인하세요.\n시작할까요?`)) return;
    const r = await fetch(`/api/align/${a}${precise ? "?precise=1" : ""}`, { method: "POST" });
    if (!r.ok) { toast(await r.text()); return; }
    const j = await r.json();
    if (a === "check") {
      alignCheck = j.ok ? `경로 점검 통과 — 관절 범위·충돌 문제 없음 (약 ${Math.round(j.duration_s)}초)` : "경로 점검 실패:";
      setTimeout(() => { alignCheck = null; }, 15000);
    } else if (a === "start" && !j.ok) {
      toast(`시작 불가: ${(j.problems || [])[0] || ""}`);
    }
  }

  function renderCalib(c) {
    const box = $("#align-result");
    const key = c ? c.id : "";
    if (box.dataset.key === key) return;
    box.dataset.key = key;
    if (!c) { box.innerHTML = `<p class="muted" style="margin:0">아직 정렬 결과가 없습니다. 지금은 모델의 기본 장착 위치를 씁니다.</p>`; return; }
    const r = c.residuals || {};
    const mm = (v) => v.map((x) => x.toFixed(1)).join(", ");
    const rows = [
      ["결과", `${esc(c.id)}${c.sim ? " (시뮬레이션)" : ""}`],
      ["모델 기본 장착 대비 회전", `${fmt(c.angle_from_nominal_deg, 2)}°`],
      ["IMU 위치 (gripper, mm)", `${mm(c.p_gi_mm)}${c.p_source !== "estimated" ? " · 기본값 (추정 불안정)" : ""}`],
      ["모델 기본 위치 (mm)", mm(c.p_nominal_mm)],
      ["위치 일관성 (데이터 반반 비교)", c.p_split_diff_mm != null ? `${fmt(c.p_split_diff_mm, 1)} mm 차이` : "—"],
      ["IMU 시간지연", `${fmt(c.lag_ms, 1)} ms`],
      ["피치 관절 0점 보정", `${fmt(c.pitch_offset_deg, 1)}°`],
      ["자이로 잔차", `${fmt(r.gyro_rms_dps, 2)} °/s`],
      ["중력 방향 잔차", `${fmt(r.gravity_rms_deg, 2)}°`],
      ["가속도 잔차", `${fmt(r.acc_rms_mps2, 3)} m/s²`],
      ["가속도계 축별 스케일", c.acc_scale && c.acc_scale.some((v) => Math.abs(v - 1) > 1e-6) ? c.acc_scale.map((v) => v.toFixed(4)).join(", ") : "보정 안 함 (정밀 정렬 필요)"],
    ];
    const warn = (c.sign_warnings || []).map((w) => `<li>${esc(w.joint)} 관절 부호가 모델과 반대일 수 있음</li>`).join("");
    box.innerHTML = `<table>${rows.map(([a, b]) => `<tr><th>${a}</th><td>${b}</td></tr>`).join("")}</table>`
      + (warn ? `<ul class="problems">${warn}</ul>` : "")
      + `<p class="muted small" style="margin:8px 0 0">새 회차부터 merged.csv에 payload 가속도 열이 추가되고, payload 질량을 넣으면 force label 열도 추가됩니다. 3D 디지털 트윈: <code>./twin.sh</code></p>`;
  }

  // ------------------------------------------------------------ standby / wake
  function renderStandby(st) {
    const arm = st.arm || {}, cf = st.cf || {};
    const f = arm.follower || {};
    const rows = [
      ["팔로워 (SO-101)", f.connected ? "ok" : "", f.connected ? "연결됨" : "연결 대기"],
      ["Crazyflie IMU", cf.state === "connected" ? "ok" : cf.state === "connecting" ? "warn" : "",
        cf.state === "connected" ? "연결됨" : cf.state === "disabled" ? "사용 안 함" : (cf.message || "연결 대기")],
      ["구동", st.moving ? "ok" : "", st.moving ? "움직임 감지" : "움직임 대기"],
    ];
    $("#standby-list").innerHTML = rows.map(([n, c, s]) => `<li class="${c}"><i></i>${esc(n)}<span>${esc(s)}</span></li>`).join("");
    const show = st.activity === "standby" && !S.dismissed;
    $("#standby").hidden = !show;
    if (S.activity !== st.activity) {
      if (st.activity === "standby") releaseWake();
      S.activity = st.activity;
    }
  }

  async function requestWake() {
    try {
      if ("wakeLock" in navigator && !S.wakeLock) {
        S.wakeLock = await navigator.wakeLock.request("screen");
        S.wakeLock.addEventListener("release", () => { S.wakeLock = null; });
      }
    } catch (_) { /* needs https or localhost */ }
  }
  function releaseWake() { if (S.wakeLock) S.wakeLock.release().catch(() => {}); }

  function onEvent(ev) {
    if (ev.type === "wake") {
      S.dismissed = false;
      $("#standby").hidden = true;
      requestWake();
      if (S.review) toast("로봇 구동 감지 — 라이브로 돌아가려면 위의 버튼을 누르세요");
      else toast("로봇 구동 감지 — 화면을 켰습니다");
    } else if (ev.type === "standby") {
      S.dismissed = false;
    } else if (ev.type === "trial_started") {
      toast(`${ev.id.split("_")[0]} 기록 시작${ev.auto ? " (자동)" : ""}`);
    } else if (ev.type === "align_done") {
      toast(`IMU 정렬 완료 (${ev.quality === "good" ? "양호" : "확인 필요"})`);
    } else if (ev.type === "trial_ready") {
      toast(ev.state === "ready" ? `${ev.id.split("_")[0]} 처리 완료 — CSV 다운로드 가능` : `${ev.id.split("_")[0]} 처리 상태: ${ev.state}`);
    }
  }

  // ------------------------------------------------------------ trials
  const STATE_TXT = { recording: ["기록 중", "rec"], processing: ["처리 중", "warn"], ready: ["완료", "ok"], error: ["오류", "warn"], empty: ["빈 회차", ""] };
  const TELEOP_MAP = { active: ["동작 중", "ok"], hold: ["자세 유지", "warn"], off: ["꺼짐", ""], script: ["정렬 동작", "rec"] };

  function clock(d) {
    const p = (n) => String(n).padStart(2, "0");
    const today = new Date().toDateString() === d.toDateString();
    return `${today ? "" : `${d.getMonth() + 1}/${d.getDate()} `}${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
  }

  function trialMeta(t) {
    const parts = [];
    if (t.duration_s != null) parts.push(fmtDur(t.duration_s));
    if (t.payload_mass_g) parts.push(`페이로드 ${fmt(t.payload_mass_g, 0)} g`);
    if (t.arm_rate_hz) parts.push(`팔 ${fmt(t.arm_rate_hz, 0)} Hz`);
    if (t.imu_rate_hz) parts.push(`IMU ${fmt(t.imu_rate_hz, 0)} Hz`);
    if (t.imu_coverage != null && t.imu_coverage < 0.95) parts.push(`<span class="warn">IMU 커버 ${fmt(t.imu_coverage * 100, 0)}%</span>`);
    if (t.lag_ms != null) parts.push(`IMU 지연 ${fmt(t.lag_ms, 1)} ms${t.lag_reliable ? "" : " (불확실)"}`);
    if (t.clock_jitter_ms != null) parts.push(`동기 ±${fmt(t.clock_jitter_ms, 2)} ms`);
    if (t.max_temp_C != null && t.max_temp_C >= 55) parts.push(`<span class="warn">최고 ${fmt(t.max_temp_C, 0)}°C</span>`);
    if (t.error) parts.push(`<span class="warn">${esc(t.error)}</span>`);
    return parts.join(" · ");
  }

  function renderTrials() {
    const box = $("#trials");
    if (box.contains(document.activeElement) && document.activeElement.tagName === "INPUT") {
      box.dataset.stale = "1";
      return;
    }
    box.dataset.stale = "";
    if (!S.trials.length) {
      box.innerHTML = `<div class="empty">아직 회차가 없습니다. 자동 회차가 켜져 있으면 팔로워가 움직일 때 기록이 시작됩니다.</div>`;
      return;
    }
    box.innerHTML = S.trials.map((t) => {
      const [txt, cls] = STATE_TXT[t.state] || [t.state, ""];
      const ready = t.state === "ready";
      const when = t.started_unix ? clock(new Date(t.started_unix * 1000)) : "";
      const viewing = S.review && S.review.id === t.id ? " viewing" : "";
      return `<div class="trial ${esc(t.state)}${viewing}" data-id="${esc(t.id)}">
        <div class="t-head"><b>${esc(t.id.split("_")[0])}</b><span class="t-time">${when}${t.auto ? " · 자동" : ""}</span><span class="badge ${cls}">${txt}</span></div>
        <div class="t-meta" data-meta>${t.state === "recording" ? "" : trialMeta(t)}</div>
        <input class="t-label" placeholder="메모 (예: payload 200g, 손목 흔들기)" value="${esc(t.label || "")}" maxlength="120">
        <div class="t-actions">
          <a class="btn primary small" href="/api/trials/${esc(t.id)}/merged.csv" download ${ready ? "" : 'aria-disabled="true" style="pointer-events:none;opacity:.45"'}>CSV 다운로드</a>
          <a class="btn ghost small" href="/api/trials/${esc(t.id)}/raw.zip" download ${t.state === "recording" ? 'aria-disabled="true" style="pointer-events:none;opacity:.45"' : ""}>원본 zip</a>
          <button class="btn ghost small" data-view ${ready ? "" : "disabled"}>그래프로 보기</button>
        </div>
      </div>`;
    }).join("");
    box.querySelectorAll(".trial").forEach((el) => {
      const id = el.dataset.id;
      const inp = el.querySelector(".t-label");
      const save = async () => {
        const t = S.trials.find((x) => x.id === id);
        if (t && (t.label || "") === inp.value) return;
        await fetch(`/api/trials/${id}/label`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ label: inp.value }) });
      };
      inp.addEventListener("change", save);
      inp.addEventListener("keydown", (e) => { if (e.key === "Enter") inp.blur(); });
      inp.addEventListener("blur", () => { if ($("#trials").dataset.stale) setTimeout(renderTrials, 0); });
      const v = el.querySelector("[data-view]");
      if (v) v.addEventListener("click", () => openReview(id));
    });
  }

  function updateRecordingTrial(rec) {
    if (!rec.recording) return;
    const el = document.querySelector(`.trial[data-id="${CSS.escape(rec.id)}"] [data-meta]`);
    if (el) el.textContent = `${fmtDur(rec.elapsed_s)} · 팔 ${rec.n_arm} · IMU ${rec.n_imu} 샘플`;
  }

  async function openReview(id) {
    const r = await fetch(`/api/trials/${id}/preview.json`);
    if (!r.ok) { toast("아직 처리 중입니다"); return; }
    S.review = { id, data: await r.json() };
    $("#review-banner").hidden = false;
    const t = S.trials.find((x) => x.id === id);
    $("#review-title").textContent = `${id.split("_")[0]} 검토 중${t && t.label ? ` — ${t.label}` : ""}`;
    $("#win-seg").hidden = true;
    renderTrials();
    requestRender(true);
    window.scrollTo({ top: 0, behavior: "smooth" });
  }
  function closeReview() {
    S.review = null;
    $("#review-banner").hidden = true;
    $("#win-seg").hidden = false;
    renderTrials();
    requestRender(true);
  }

  // ------------------------------------------------------------ controls
  function selectJoint(j) {
    S.sel = j;
    document.querySelectorAll("#joint-seg button").forEach((b, i) => b.classList.toggle("on", i === j));
    $("#jname").textContent = S.joints[j];
    $("#jname2").textContent = S.joints[j];
    if (S.status) renderJointTable(S.status.joints);
    requestRender(true);
  }

  function buildJointSeg() {
    const jc = jointColors();
    const seg = $("#joint-seg");
    seg.innerHTML = S.joints.map((n, i) => `<button role="tab" class="${i === S.sel ? "on" : ""}"><span class="sw" style="background:${jc[i]}"></span>${esc(n)}</button>`).join("");
    seg.querySelectorAll("button").forEach((b, i) => b.addEventListener("click", () => selectJoint(i)));
  }

  $("#win-seg").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
    S.win = +b.dataset.w;
    $("#win-seg").querySelectorAll("button").forEach((x) => x.classList.toggle("on", x === b));
    requestRender(true);
  }));
  $("#btn-live").addEventListener("click", closeReview);
  $("#payload").addEventListener("change", async (e) => {
    const r = await fetch("/api/payload", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ mass_g: e.target.value }) });
    const j = await r.json();
    toast(j.mass_g ? `페이로드 ${j.mass_g} g — 새 회차부터 force label 생성` : "페이로드 질량 해제");
  });
  $("#btn-peek").addEventListener("click", () => { S.dismissed = true; $("#standby").hidden = true; });
  $("#btn-rec").addEventListener("click", async () => {
    const on = S.status && S.status.rec && S.status.rec.recording;
    await fetch(`/api/record/${on ? "stop" : "start"}`, { method: "POST" });
  });
  $("#auto").addEventListener("change", async (e) => {
    await fetch("/api/auto", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ on: e.target.checked }) });
  });

  let toastTimer = null;
  function toast(msg) {
    const t = $("#toast");
    t.textContent = msg;
    t.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.remove("show"), 3500);
  }

  // ------------------------------------------------------------ websocket
  let retry = 0;
  function connect() {
    const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
    ws.onopen = () => { retry = 0; $("#conn-banner").hidden = true; };
    ws.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      switch (msg.type) {
        case "reload": location.reload(); return;
        case "payload": { const p = $("#payload"); if (document.activeElement !== p) p.value = msg.mass_g ?? ""; break; }
        case "hello": {
          if (S.assetVersion && msg.asset_version !== S.assetVersion) { location.reload(); return; }
          S.assetVersion = msg.asset_version;
          $("#payload").value = msg.payload_mass_g ?? "";
          const changed = JSON.stringify(S.joints) !== JSON.stringify(msg.joints);
          S.joints = msg.joints; S.control = msg.control_allowed; S.urls = msg.urls; S.sim = msg.sim;
          S.liveKeep = (msg.live_window_s || 15) + 1;
          if (S.sel >= S.joints.length) S.sel = 0;
          resetBuffers();
          if (changed || !charts.pos) { buildJointSeg(); buildCharts(); $("#jtable tbody").innerHTML = ""; }
          $("#mode").textContent = msg.sim ? "SO-101 × Crazyflie · 시뮬레이션" : "SO-101 × Crazyflie";
          $("#urls").innerHTML = (msg.urls || []).map((u) => `<div><code>${esc(u)}</code></div>`).join("") || `<span class="muted">LAN 주소를 찾지 못했습니다</span>`;
          $("#align-ctl").dataset.key = "";
          break;
        }
        case "status": renderStatus(msg); break;
        case "data": onData(msg); break;
        case "trials": S.trials = msg.trials; renderTrials(); break;
        default: onEvent(msg);
      }
    };
    ws.onclose = () => {
      $("#conn-banner").hidden = false;
      setTimeout(connect, Math.min(5000, 500 * 2 ** retry++));
    };
  }

  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { if (S.joints.length) { buildJointSeg(); buildCharts(); } });
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "visible" && S.activity === "active") requestWake();
  });
  connect();
})();
