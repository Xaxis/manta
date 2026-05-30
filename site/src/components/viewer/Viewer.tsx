import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls, Html, ContactShadows, Environment } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

// ---------------------------------------------------------------
// MANTA viewer. The single GLB holds two morph passes baked from
// sim/build.py: deployment (frames 0..deploy_n-1) and a flight-
// control demo (frames deploy_n..total-1, flaperons + pilot weight-
// shift). The control bank/pitch attitude + telemetry come from
// control.json (derived from the stability/control derivatives).
// A "Flow field" toggle reveals the baked surface-Cp + streamlines.
// ---------------------------------------------------------------

const MODEL_URL = "/models/v3/manta.glb";
const TELEMETRY_URL = "/models/v3/telemetry.json";
const CONTROL_URL = "/models/v3/control.json";

type Telemetry = {
  settle: { V_glide: number; glide_ratio: number };
  series: {
    deploy: number[]; V: number[]; gamma_deg: number[];
    n_load: number[]; glide_ratio: number[];
  };
};

type ControlPt = { tc: number; bank: number; pitch: number; roll_rate: number;
                   da: number; de: number; n: number };
type Control = { deploy_frames: number; control_frames: number;
                 total_frames: number; duration_s: number; series: ControlPt[] };

function makeFlowMaterial() {
  return new THREE.ShaderMaterial({
    transparent: true, depthWrite: false,
    uniforms: { uTime: { value: 0 } },
    vertexShader: `varying vec2 vUv; void main(){ vUv=uv; gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}`,
    fragmentShader: `
      uniform float uTime; varying vec2 vUv;
      vec3 ramp(float t){ vec3 a=vec3(0.18,0.42,0.98),b=vec3(0.20,0.95,0.62),c=vec3(1.0,0.52,0.12);
        return t<0.5?mix(a,b,t*2.0):mix(b,c,(t-0.5)*2.0);}
      void main(){ float s=clamp(vUv.y,0.0,1.0);
        float p=fract(vUv.x*6.0-uTime*(0.55+s*1.5));
        float br=smoothstep(0.0,0.12,p)*(1.0-smoothstep(0.32,0.62,p));
        gl_FragColor=vec4(ramp(s)*(0.30+1.05*br), 0.30+0.6*br);}`,
  });
}

function lerp(a: number, b: number, t: number) { return a + (b - a) * t; }
function sampleControl(series: ControlPt[], p: number): ControlPt {
  if (series.length === 0) return { tc: 0, bank: 0, pitch: 0, roll_rate: 0, da: 0, de: 0, n: 1 };
  const x = THREE.MathUtils.clamp(p, 0, 1) * (series.length - 1);
  const i = Math.floor(x), f = x - i, j = Math.min(i + 1, series.length - 1);
  const a = series[i], b = series[j];
  return { tc: p, bank: lerp(a.bank, b.bank, f), pitch: lerp(a.pitch, b.pitch, f),
    roll_rate: lerp(a.roll_rate, b.roll_rate, f), da: lerp(a.da, b.da, f),
    de: lerp(a.de, b.de, f), n: lerp(a.n, b.n, f) };
}

function MantaModel({ deployRef, flowMode, control, controlRef }: {
  deployRef: React.MutableRefObject<number>;
  flowMode: boolean;
  control: Control | null;
  controlRef: React.MutableRefObject<number | null>;
}) {
  const gltf = useLoader(GLTFLoader, MODEL_URL);
  const flowMat = useMemo(() => makeFlowMaterial(), []);
  const attitude = useRef<THREE.Group>(null);

  const { mixer, duration, parts } = useMemo(() => {
    const mx = new THREE.AnimationMixer(gltf.scene);
    const clip = gltf.animations[0];
    if (clip) mx.clipAction(clip).play();
    const parts: { pressure?: THREE.Object3D; flow?: THREE.Object3D;
      skinMats: THREE.MeshStandardMaterial[] } = { skinMats: [] };
    gltf.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (o.name === "PRESSURE") parts.pressure = o;
      if (o.name === "FLOW") parts.flow = o;
      if (mesh.isMesh) {
        mesh.castShadow = true; mesh.frustumCulled = false;
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        mats.forEach((mm) => { const sm = mm as THREE.MeshStandardMaterial;
          if (sm && sm.name === "skin") parts.skinMats.push(sm); });
        if (o.name === "FLOW") mesh.material = flowMat;
      }
    });
    return { mixer: mx, duration: clip ? clip.duration : 1, parts };
  }, [gltf, flowMat]);

  useEffect(() => {
    if (parts.pressure) parts.pressure.visible = flowMode;
    if (parts.flow) parts.flow.visible = flowMode;
    parts.skinMats.forEach((sm) => { sm.transparent = true;
      sm.opacity = flowMode ? 0.06 : 0.32; sm.needsUpdate = true; });
  }, [flowMode, parts]);

  useFrame((_, dt) => {
    flowMat.uniforms.uTime.value += dt;
    const total = control?.total_frames ?? 1;
    const deployN = control?.deploy_frames ?? total;
    const cp = controlRef.current;
    if (cp != null && control) {
      // control demo: scrub the control portion + bank/pitch the craft
      const frame = (deployN - 1) + cp * (total - deployN);
      mixer.setTime((frame / (total - 1)) * duration);
      const s = sampleControl(control.series, cp);
      if (attitude.current) attitude.current.rotation.set(
        THREE.MathUtils.degToRad(s.bank), 0, THREE.MathUtils.degToRad(s.pitch));
    } else {
      // deployment scrub (slider maps onto the deploy frames only)
      if (attitude.current) attitude.current.rotation.set(0, 0, 0);
      const d = THREE.MathUtils.clamp(deployRef.current, 0, 1);
      const frame = d * (deployN - 1);
      mixer.setTime((frame / (total - 1)) * duration);
    }
  });

  return <group ref={attitude}><primitive object={gltf.scene} /></group>;
}

function Scene(props: { deployRef: React.MutableRefObject<number>; flowMode: boolean;
  control: Control | null; controlRef: React.MutableRefObject<number | null>; }) {
  return (
    <>
      <hemisphereLight args={["#dfe9ff", "#10131c", 0.55]} />
      <directionalLight position={[5, 8, 4]} intensity={2.4} castShadow
        shadow-mapSize-width={2048} shadow-mapSize-height={2048}
        shadow-camera-near={0.5} shadow-camera-far={30} shadow-camera-left={-6}
        shadow-camera-right={6} shadow-camera-top={6} shadow-camera-bottom={-6} />
      <directionalLight position={[-4, 3, -5]} intensity={0.45} color="#9fb8ff" />
      <Suspense fallback={<Html center><div className="text-zinc-300 text-sm">Loading model…</div></Html>}>
        <MantaModel {...props} />
        <Environment preset="city" />
      </Suspense>
      <ContactShadows position={[0, -1.05, 0]} opacity={0.45} scale={14} blur={2.4} far={4} color="#000010" />
      <gridHelper args={[14, 28, "#1a3a8e", "#1d2230"]} position={[0, -1.05, 0]} />
    </>
  );
}

const PLAY_MS = 2200;
function easeDeploy(t: number): number {
  if (t < 0.55) { const tt = t / 0.55; return tt * tt * (3 - 2 * tt) * 0.45; }
  const tt = (t - 0.55) / 0.45; return 0.45 + tt * tt * (3 - 2 * tt) * 0.55;
}

export default function Viewer({ height = 620 }: { height?: number }) {
  const [deployState, setDeployState] = useState(1.0);
  const [playing, setPlaying] = useState(false);
  const [flowMode, setFlowMode] = useState(false);
  const [flying, setFlying] = useState(false);
  const [ctrlProg, setCtrlProg] = useState(0);
  const [autoRotate, setAutoRotate] = useState(true);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const [control, setControl] = useState<Control | null>(null);
  const deployRef = useRef(1.0);
  const controlRef = useRef<number | null>(null);
  deployRef.current = deployState;

  useEffect(() => { fetch(TELEMETRY_URL).then((r) => r.json()).then(setTelem).catch(() => {}); }, []);
  useEffect(() => { fetch(CONTROL_URL).then((r) => r.json()).then(setControl).catch(() => {}); }, []);

  // deployment autoplay
  useEffect(() => {
    if (!playing) return;
    let raf = 0; const start = performance.now(); setDeployState(0);
    const tick = () => {
      const t = (performance.now() - start) / PLAY_MS;
      if (t >= 1) { setDeployState(1); setPlaying(false); }
      else { setDeployState(easeDeploy(t)); raf = requestAnimationFrame(tick); }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing]);

  // control demo loop (loops the maneuver)
  useEffect(() => {
    if (!flying || !control) { controlRef.current = null; return; }
    setDeployState(1); setFlowMode(false);
    let raf = 0; const dur = control.duration_s * 1000; const start = performance.now();
    const tick = () => {
      const p = ((performance.now() - start) % dur) / dur;
      controlRef.current = p; setCtrlProg(p);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => { cancelAnimationFrame(raf); controlRef.current = null; };
  }, [flying, control]);

  const live = useMemo(() => {
    if (!telem) return null;
    if (deployState >= 0.99) return { V: telem.settle.V_glide,
      gamma: -Math.atan2(1, telem.settle.glide_ratio) * (180 / Math.PI),
      n: 1.0, ld: telem.settle.glide_ratio };
    const d = telem.series.deploy; let idx = 0;
    for (let i = 0; i < d.length; i++) { if (d[i] <= deployState) idx = i; else break; }
    return { V: telem.series.V[idx], gamma: telem.series.gamma_deg[idx],
      n: telem.series.n_load[idx], ld: telem.series.glide_ratio[idx] };
  }, [telem, deployState]);

  const ctrlNow = useMemo(() =>
    control ? sampleControl(control.series, ctrlProg) : null, [control, ctrlProg]);

  const btn = "rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition";

  const phaseLabel = (() => {
    if (flying) {
      const m = (ctrlNow?.bank ?? 0) < -6 ? "ROLL LEFT" : (ctrlNow?.bank ?? 0) > 6 ? "ROLL RIGHT"
        : (ctrlNow?.pitch ?? 0) > 3 ? "PITCH FLARE" : "WINGS LEVEL";
      return `FLIGHT CONTROL — ${m} · differential flaperon + pilot weight-shift`;
    }
    if (flowMode) return "AERO FIELD — surface Cp + streamlines (illustrative, not CFD)";
    if (deployState < 0.04) return "STOWED — wingsuit-tucked freefall posture";
    if (deployState < 0.46) return "PHASE A — pneumatic yokes spread arms + legs";
    if (deployState < 0.96) return "PHASE B — CO₂ telescopes the tip booms";
    return "DEPLOYED — double-surface wing · best-glide trim";
  })();

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas shadows dpr={[1, 2]} camera={{ position: [4.2, 2.4, 5.2], fov: 34, near: 0.05, far: 80 }}
        onCreated={({ scene }) => { scene.background = new THREE.Color("#070809"); }}>
        <Scene deployRef={deployRef} flowMode={flowMode} control={control} controlRef={controlRef} />
        <OrbitControls enableDamping dampingFactor={0.06} autoRotate={autoRotate && !flying}
          autoRotateSpeed={0.5} minDistance={2.0} maxDistance={24} target={[0, 0, 0]} />
      </Canvas>

      {/* controls */}
      <div className="absolute top-3 right-3 flex flex-col gap-2 max-w-[220px]">
        <button onClick={() => { setFlowMode(false); setFlying(false); setPlaying(true); }} disabled={playing}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 disabled:opacity-40 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition">
          {playing ? "Deploying…" : "▶ Play deployment"}
        </button>
        <button onClick={() => { setFlowMode(false); setPlaying(false); setFlying((v) => !v); }}
          className={flying ? "rounded-md bg-emerald-500/90 hover:bg-emerald-500 border border-emerald-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
            : btn}>
          {flying ? "✓ Flight control ON" : "🕹 Fly it (control)"}
        </button>
        <button onClick={() => (flowMode ? setFlowMode(false) : (setFlying(false), setPlaying(false), setDeployState(1), setFlowMode(true)))}
          className={flowMode ? "rounded-md bg-sky-500/90 hover:bg-sky-500 border border-sky-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition" : btn}>
          {flowMode ? "✓ Flow field ON" : "≈ Flow field"}
        </button>
        <button onClick={() => { setFlying(false); setFlowMode(false); setPlaying(false); setDeployState(0); }} className={btn}>⤴ Stowed (0%)</button>
        <button onClick={() => { setFlying(false); setPlaying(false); setDeployState(1); }} className={btn}>⤵ Deployed (100%)</button>
        <button onClick={() => setAutoRotate((v) => !v)} className={btn}>{autoRotate ? "Stop orbit" : "Auto-orbit"}</button>
      </div>

      {/* flight-control telemetry */}
      {flying && ctrlNow && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/75 border border-emerald-900/60 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm">
          <div className="text-emerald-400/80 uppercase tracking-wider text-[10px]">flight control · pilot input → response</div>
          <Telem label="bank angle φ" value={`${ctrlNow.bank.toFixed(0)}°`} />
          <Telem label="roll rate p" value={`${ctrlNow.roll_rate.toFixed(0)} °/s`} />
          <Telem label="load factor n" value={`${ctrlNow.n.toFixed(2)} g`} />
          <Telem label="pitch θ" value={`${ctrlNow.pitch.toFixed(0)}°`} />
          <div className="mt-1 pt-1 border-t border-zinc-800 text-[10px] text-zinc-500">
            aileron {ctrlNow.da >= 0 ? "+" : ""}{(ctrlNow.da * 22).toFixed(0)}° · elevator {(ctrlNow.de * 18).toFixed(0)}°
          </div>
        </div>
      )}

      {/* deploy flight-physics telemetry */}
      {live && !flowMode && !flying && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/70 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm">
          <div className="text-zinc-500 uppercase tracking-wider text-[10px]">flight physics · live</div>
          <Telem label="airspeed" value={`${live.V.toFixed(1)} m/s`} />
          <Telem label="flight path γ" value={`${live.gamma.toFixed(1)}°`} />
          <Telem label="load factor" value={`${live.n.toFixed(2)} g`} />
          <Telem label="glide ratio" value={live.ld > 0 && live.ld < 40 ? `${live.ld.toFixed(1)} : 1` : "—"} />
        </div>
      )}

      {/* pressure legend */}
      {flowMode && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/75 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm max-w-[230px]">
          <div className="text-zinc-400 uppercase tracking-wider text-[10px]">surface pressure · Cp</div>
          <div className="h-2 w-full rounded" style={{ background: "linear-gradient(90deg,#1a33d9,#19bff2,#34d94d,#fad926,#f22620)" }} />
          <div className="flex justify-between text-[10px] text-zinc-500"><span>suction (lift)</span><span>stagnation</span></div>
          <div className="text-[10px] text-zinc-500 leading-snug pt-1">Weissinger span-load + thin-airfoil Cp — illustrative, not CFD.</div>
        </div>
      )}

      {/* deploy scrubber */}
      <div className="absolute bottom-3 left-3 right-3 flex items-center gap-3 bg-zinc-900/85 border border-zinc-800 px-3 py-2 rounded-md text-xs">
        <span className="text-zinc-500 whitespace-nowrap">{flying ? "maneuver" : "deploy"}</span>
        <input type="range" min={0} max={1} step={0.005}
          value={flying ? ctrlProg : deployState} disabled={flowMode || flying}
          onChange={(e) => { setDeployState(parseFloat(e.target.value)); setPlaying(false); }}
          className="flex-1 accent-orange-500 disabled:opacity-50" aria-label="state" />
        <span className="mono text-zinc-300 tabular-nums w-10 text-right">{((flying ? ctrlProg : deployState) * 100).toFixed(0)}%</span>
        <span className="hidden md:inline text-zinc-400 ml-2 text-[11px] uppercase tracking-wider">{phaseLabel}</span>
      </div>
      <div className="absolute bottom-[3.4rem] left-3 text-[10px] text-zinc-600">
        planform S 6.5 m² · b 6.3 m · AR 6.1 · double-surface wing · deploy + flight-control demo
      </div>
    </div>
  );
}

function Telem({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between gap-4">
      <span className="text-zinc-500">{label}</span>
      <span className="mono text-zinc-200 tabular-nums">{value}</span>
    </div>
  );
}
