import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useLoader, useThree } from "@react-three/fiber";
import { OrbitControls, Html, ContactShadows, Environment, Sky, Cloud, Clouds } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

// ---------------------------------------------------------------
// MANTA viewer. Deployment morph + four control-basis morph targets
// (rollL/R, pitchU/D). "Fly it" runs a real reduced-order flight
// model (control.json): you steer with the keyboard, the EOM
// integrate airspeed / bank / load factor / AoA (with the alpha-
// limiter), the flaperons deflect + the pilot weight-shifts live,
// and the craft actually flies over procedural terrain under a sky.
// ---------------------------------------------------------------

const MODEL_URL = "/models/v3/manta.glb";
const TELEMETRY_URL = "/models/v3/telemetry.json";
const CONTROL_URL = "/models/v3/control.json";

type FlightModel = {
  S: number; AR: number; CL_alpha: number; CD0: number; e: number; mass: number;
  g: number; rho: number; alpha0_deg: number; alpha_trim_deg: number;
  alpha_limit_deg: number; alpha_stall_deg: number; CLmax: number; V_trim: number;
  roll_rate_max_dps: number; bank_limit_deg: number;
};
type Control = { model: FlightModel; deploy_frames: number; ctrl_keys: string[] };
type Telemetry = { settle: { V_glide: number; glide_ratio: number };
  series: { deploy: number[]; V: number[]; gamma_deg: number[]; n_load: number[]; glide_ratio: number[] } };
type Flight = { V: number; gamma: number; psi: number; phi: number; alpha: number;
  h: number; x: number; z: number; n: number; limited: boolean };

const d2r = THREE.MathUtils.degToRad, r2d = THREE.MathUtils.radToDeg, clamp = THREE.MathUtils.clamp, mlerp = THREE.MathUtils.lerp;
const ALT0 = 220, TSIZE = 9000, TSEG = 160, SKY_FOG = new THREE.Color("#aac4e6");
// chase camera: sits behind + above, heading-relative, so it stays behind a
// turning craft; smooth time-constant follow.
const CHASE_BACK = 20, CHASE_UP = 6.5, CHASE_TAU = 0.18;
const _fwd = new THREE.Vector3(), _desired = new THREE.Vector3(), _look = new THREE.Vector3();

// ---- procedural terrain height (shared by mesh + ground-contact) ----
function thash(x: number, z: number) { const s = Math.sin(x * 127.1 + z * 311.7) * 43758.5453; return s - Math.floor(s); }
function vnoise(x: number, z: number) {
  const xi = Math.floor(x), zi = Math.floor(z), xf = x - xi, zf = z - zi;
  const u = xf * xf * (3 - 2 * xf), v = zf * zf * (3 - 2 * zf);
  return mlerp(mlerp(thash(xi, zi), thash(xi + 1, zi), u), mlerp(thash(xi, zi + 1), thash(xi + 1, zi + 1), u), v);
}
function fbm(x: number, z: number) { let s = 0, a = 1, f = 1; for (let i = 0; i < 4; i++) { s += a * vnoise(x * f, z * f); f *= 2; a *= 0.5; } return s / 1.875; }
function terrainH(x: number, z: number) { return (fbm(x * 0.0011, z * 0.0011) - 0.45) * 150 + (fbm(x * 0.006, z * 0.006) - 0.5) * 20; }

function Terrain({ visible }: { visible: boolean }) {
  const geo = useMemo(() => {
    const g = new THREE.PlaneGeometry(TSIZE, TSIZE, TSEG, TSEG);
    g.rotateX(-Math.PI / 2);
    const p = g.attributes.position as THREE.BufferAttribute;
    const col: number[] = [];
    const low = new THREE.Color("#5a4a32"), mid = new THREE.Color("#3f6b35"),
      hi = new THREE.Color("#8a8f96"), snow = new THREE.Color("#e9eef4");
    const c = new THREE.Color();
    for (let i = 0; i < p.count; i++) {
      const x = p.getX(i), z = p.getZ(i), y = terrainH(x, z);
      p.setY(i, y);
      const t = clamp((y + 80) / 200, 0, 1);
      if (t < 0.45) c.copy(low).lerp(mid, t / 0.45);
      else if (t < 0.75) c.copy(mid).lerp(hi, (t - 0.45) / 0.30);
      else c.copy(hi).lerp(snow, (t - 0.75) / 0.25);
      col.push(c.r, c.g, c.b);
    }
    g.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
    g.computeVertexNormals();
    return g;
  }, []);
  return (
    <mesh geometry={geo} visible={visible} receiveShadow>
      <meshStandardMaterial vertexColors roughness={0.96} metalness={0} />
    </mesh>
  );
}

function makeFlowMaterial() {
  return new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, uniforms: { uTime: { value: 0 } },
    vertexShader: `varying vec2 vUv; void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}`,
    fragmentShader: `uniform float uTime;varying vec2 vUv;
      vec3 ramp(float t){vec3 a=vec3(0.18,0.42,0.98),b=vec3(0.20,0.95,0.62),c=vec3(1.0,0.52,0.12);
        return t<0.5?mix(a,b,t*2.0):mix(b,c,(t-0.5)*2.0);}
      void main(){float s=clamp(vUv.y,0.0,1.0);float p=fract(vUv.x*6.0-uTime*(0.55+s*1.5));
        float br=smoothstep(0.0,0.12,p)*(1.0-smoothstep(0.32,0.62,p));
        gl_FragColor=vec4(ramp(s)*(0.30+1.05*br),0.30+0.6*br);}`,
  });
}

function freshFlight(M?: FlightModel): Flight {
  return { V: M?.V_trim ?? 18, gamma: d2r(-3.5), psi: 0, phi: 0,
    alpha: d2r(M?.alpha_trim_deg ?? 8), h: 0, x: 0, z: 0, n: 1, limited: false };
}
function stepFlight(f: Flight, M: FlightModel, rollIn: number, pitchIn: number, dt: number) {
  f.phi += rollIn * d2r(M.roll_rate_max_dps) * dt;
  f.phi = clamp(f.phi, -d2r(M.bank_limit_deg), d2r(M.bank_limit_deg));
  let aCmd = d2r(M.alpha_trim_deg) + pitchIn * d2r(6.5);
  f.limited = aCmd > d2r(M.alpha_limit_deg);
  aCmd = clamp(aCmd, d2r(-2), d2r(M.alpha_limit_deg));
  f.alpha += (aCmd - f.alpha) * Math.min(1, dt / 0.15);
  const a0 = d2r(M.alpha0_deg);
  const CL = clamp(M.CL_alpha * (f.alpha - a0), -0.25, M.CLmax);
  const CD = M.CD0 + (CL * CL) / (Math.PI * M.e * M.AR);
  const q = 0.5 * M.rho * f.V * f.V, W = M.mass * M.g;
  const L = q * M.S * CL, D = q * M.S * CD;
  f.n = L / W;
  const cg = Math.max(Math.cos(f.gamma), 0.3);
  f.gamma += ((L * Math.cos(f.phi) - W * Math.cos(f.gamma)) / (M.mass * f.V)) * dt;
  f.V += ((-D - W * Math.sin(f.gamma)) / M.mass) * dt;
  f.psi += ((L * Math.sin(f.phi)) / (M.mass * f.V * cg)) * dt;
  f.V = clamp(f.V, 11, 48);
  f.gamma = clamp(f.gamma, -d2r(55), d2r(28));
  const Vh = f.V * Math.cos(f.gamma);
  f.h += f.V * Math.sin(f.gamma) * dt;
  f.x += Vh * Math.cos(f.psi) * dt;
  f.z += -Vh * Math.sin(f.psi) * dt;
  // ground contact -> relaunch
  if (ALT0 + f.h <= terrainH(f.x, f.z) + 6) {
    const keepPsi = f.psi;
    Object.assign(f, freshFlight(M)); f.psi = keepPsi;
  }
}

function MantaModel({ flyRef, flying, flightRef, inputRef, control, deployRef, flowMode }: {
  flyRef: React.MutableRefObject<boolean>; flying: boolean;
  flightRef: React.MutableRefObject<Flight>;
  inputRef: React.MutableRefObject<{ roll: number; pitch: number }>;
  control: Control | null; deployRef: React.MutableRefObject<number>; flowMode: boolean;
}) {
  const gltf = useLoader(GLTFLoader, MODEL_URL);
  const flowMat = useMemo(() => makeFlowMaterial(), []);
  const craft = useRef<THREE.Group>(null);
  const attitude = useRef<THREE.Group>(null);
  const { camera, scene } = useThree() as any;

  const { mixer, duration, parts, morphs } = useMemo(() => {
    const mx = new THREE.AnimationMixer(gltf.scene);
    const clip = gltf.animations[0]; if (clip) mx.clipAction(clip).play();
    const parts: { pressure?: THREE.Object3D; flow?: THREE.Object3D; skinMats: THREE.MeshStandardMaterial[] } = { skinMats: [] };
    const morphs: THREE.Mesh[] = [];
    gltf.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (o.name === "PRESSURE") parts.pressure = o;
      if (o.name === "FLOW") parts.flow = o;
      if (mesh.isMesh) {
        mesh.castShadow = true; mesh.frustumCulled = false;
        if (mesh.morphTargetInfluences && mesh.morphTargetDictionary) morphs.push(mesh);
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        mats.forEach((mm) => { const sm = mm as THREE.MeshStandardMaterial; if (sm && sm.name === "skin") parts.skinMats.push(sm); });
        if (o.name === "FLOW") mesh.material = flowMat;
      }
    });
    return { mixer: mx, duration: clip ? clip.duration : 1, parts, morphs };
  }, [gltf, flowMat]);

  useEffect(() => {
    if (parts.pressure) parts.pressure.visible = flowMode;
    if (parts.flow) parts.flow.visible = flowMode;
    parts.skinMats.forEach((sm) => { sm.transparent = true; sm.opacity = flowMode ? 0.06 : 0.32; sm.needsUpdate = true; });
  }, [flowMode, parts]);

  // entering / leaving flight: world placement, fog, sky background, camera
  useEffect(() => {
    if (flying) {
      flightRef.current = freshFlight(control?.model);
      scene.fog = new THREE.Fog(SKY_FOG, 120, 3200);
      scene.background = null;
      // start the chase camera behind + above the craft (it starts at psi=0,
      // facing +x, at the origin / ALT0). near/far bracket craft + sky dome.
      camera.position.set(-CHASE_BACK, ALT0 + CHASE_UP, 0);
      camera.lookAt(0, ALT0, 0);
      camera.near = 1; camera.far = 30000; camera.updateProjectionMatrix();
    } else {
      scene.fog = null; scene.background = new THREE.Color("#070809");
      if (craft.current) { craft.current.position.set(0, 0, 0); craft.current.rotation.set(0, 0, 0); }
      if (attitude.current) attitude.current.rotation.set(0, 0, 0);
      camera.position.set(4.2, 2.4, 5.2); camera.near = 0.05; camera.far = 80; camera.updateProjectionMatrix();
    }
  }, [flying]); // eslint-disable-line

  const deployName = control ? `frame_${String(control.deploy_frames - 1).padStart(3, "0")}` : "frame_039";

  useFrame((_, dtRaw) => {
    flowMat.uniforms.uTime.value += dtRaw;
    const dt = Math.min(dtRaw, 0.05);
    if (flyRef.current && control) {
      stepFlight(flightRef.current, control.model, inputRef.current.roll, inputRef.current.pitch, dt);
      const f = flightRef.current;
      if (craft.current) { craft.current.position.set(f.x, ALT0 + f.h, f.z); craft.current.rotation.set(0, f.psi, 0); }
      if (attitude.current) attitude.current.rotation.set(f.phi, 0, f.gamma + f.alpha);
      const ri = inputRef.current.roll, pi = inputRef.current.pitch;
      morphs.forEach((mesh) => {
        const infl = mesh.morphTargetInfluences!, dict = mesh.morphTargetDictionary!;
        for (let i = 0; i < infl.length; i++) infl[i] = 0;
        const set = (n: string, v: number) => { if (dict[n] != null) infl[dict[n]] = v; };
        set(deployName, 1);
        set("ctrl_rollL", Math.max(0, -ri)); set("ctrl_rollR", Math.max(0, ri));
        set("ctrl_pitchU", Math.max(0, pi)); set("ctrl_pitchD", Math.max(0, -pi));
      });
      // chase camera: stay behind + above, heading-relative, smooth follow
      const cp = craft.current!.position;
      _fwd.set(Math.cos(f.psi), 0, -Math.sin(f.psi));
      _desired.copy(cp).addScaledVector(_fwd, -CHASE_BACK); _desired.y += CHASE_UP;
      camera.position.lerp(_desired, 1 - Math.exp(-dt / CHASE_TAU));
      camera.lookAt(_look.set(cp.x, cp.y + 1.2, cp.z));
    } else {
      mixer.setTime(clamp(deployRef.current, 0, 1) * duration);
    }
  });

  return (
    <group ref={craft}>
      <group ref={attitude}><primitive object={gltf.scene} /></group>
    </group>
  );
}

function Scene(props: {
  flyRef: React.MutableRefObject<boolean>; flying: boolean;
  flightRef: React.MutableRefObject<Flight>; inputRef: React.MutableRefObject<{ roll: number; pitch: number }>;
  control: Control | null; deployRef: React.MutableRefObject<number>; flowMode: boolean;
}) {
  return (
    <>
      <hemisphereLight args={["#dfe9ff", "#10131c", props.flying ? 0.8 : 0.55]} />
      <directionalLight position={[180, 320, 120]} intensity={2.6} castShadow
        shadow-mapSize-width={2048} shadow-mapSize-height={2048} shadow-camera-near={0.5}
        shadow-camera-far={props.flying ? 1200 : 30}
        shadow-camera-left={-(props.flying ? 200 : 6)} shadow-camera-right={props.flying ? 200 : 6}
        shadow-camera-top={props.flying ? 200 : 6} shadow-camera-bottom={-(props.flying ? 200 : 6)} />
      <directionalLight position={[-4, 3, -5]} intensity={0.35} color="#9fb8ff" />
      <Suspense fallback={<Html center><div className="text-zinc-300 text-sm">Loading model…</div></Html>}>
        <MantaModel {...props} />
        {!props.flying && <Environment preset="city" />}
      </Suspense>
      {props.flying && (
        <>
          <Sky distance={20000} sunPosition={[180, 90, 120]} turbidity={6} rayleigh={2} mieCoefficient={0.005} mieDirectionalG={0.8} />
          <Terrain visible />
          <Clouds material={THREE.MeshBasicMaterial} limit={80}>
            <Cloud seed={1} bounds={[1400, 60, 1400]} segments={26} volume={420} color="#eef3fb" position={[200, ALT0 + 140, -300]} opacity={0.55} />
            <Cloud seed={7} bounds={[1600, 60, 1600]} segments={26} volume={480} color="#e6edf7" position={[-600, ALT0 + 90, 500]} opacity={0.5} />
          </Clouds>
        </>
      )}
      {!props.flying && (
        <>
          <ContactShadows position={[0, -1.05, 0]} opacity={0.4} scale={14} blur={2.4} far={4} color="#000010" />
          <gridHelper args={[14, 28, "#1a3a8e", "#1d2230"]} position={[0, -1.05, 0]} />
        </>
      )}
      {/* OrbitControls only for inspection; flight uses the chase camera */}
      {!props.flying && (
        <OrbitControls makeDefault enableDamping dampingFactor={0.06} autoRotate
          autoRotateSpeed={0.5} minDistance={2.0} maxDistance={24} target={[0, 0, 0]} />
      )}
    </>
  );
}

const PLAY_MS = 2200;
function easeDeploy(t: number) {
  if (t < 0.55) { const tt = t / 0.55; return tt * tt * (3 - 2 * tt) * 0.45; }
  const tt = (t - 0.55) / 0.45; return 0.45 + tt * tt * (3 - 2 * tt) * 0.55;
}

export default function Viewer({ height = 620 }: { height?: number }) {
  const [deployState, setDeployState] = useState(1.0);
  const [playing, setPlaying] = useState(false);
  const [flowMode, setFlowMode] = useState(false);
  const [flying, setFlying] = useState(false);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const [control, setControl] = useState<Control | null>(null);
  const [hud, setHud] = useState<Flight>(freshFlight());

  const deployRef = useRef(1.0); deployRef.current = deployState;
  const flyRef = useRef(false); flyRef.current = flying;
  const flightRef = useRef<Flight>(freshFlight());
  const inputRef = useRef({ roll: 0, pitch: 0 });
  const keys = useRef<Record<string, boolean>>({});

  useEffect(() => { fetch(TELEMETRY_URL).then((r) => r.json()).then(setTelem).catch(() => {}); }, []);
  useEffect(() => { fetch(CONTROL_URL).then((r) => r.json()).then(setControl).catch(() => {}); }, []);

  useEffect(() => {
    if (!playing) return;
    let raf = 0; const start = performance.now(); setDeployState(0);
    const tick = () => { const t = (performance.now() - start) / PLAY_MS;
      if (t >= 1) { setDeployState(1); setPlaying(false); } else { setDeployState(easeDeploy(t)); raf = requestAnimationFrame(tick); } };
    raf = requestAnimationFrame(tick); return () => cancelAnimationFrame(raf);
  }, [playing]);

  // keyboard input smoothing + HUD refresh
  useEffect(() => {
    const arrows = ["arrowup", "arrowdown", "arrowleft", "arrowright"];
    const down = (e: KeyboardEvent) => { keys.current[e.key.toLowerCase()] = true; if (flyRef.current && arrows.includes(e.key.toLowerCase())) e.preventDefault(); };
    const up = (e: KeyboardEvent) => { keys.current[e.key.toLowerCase()] = false; };
    window.addEventListener("keydown", down); window.addEventListener("keyup", up);
    let raf = 0;
    const loop = () => {
      const k = keys.current;
      const rT = (k["arrowright"] || k["d"] ? 1 : 0) - (k["arrowleft"] || k["a"] ? 1 : 0);
      const pT = (k["arrowup"] || k["w"] ? 1 : 0) - (k["arrowdown"] || k["s"] ? 1 : 0);
      const i = inputRef.current; i.roll += (rT - i.roll) * 0.15; i.pitch += (pT - i.pitch) * 0.15;
      if (flyRef.current) setHud({ ...flightRef.current });
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => { window.removeEventListener("keydown", down); window.removeEventListener("keyup", up); cancelAnimationFrame(raf); };
  }, []);

  const startFly = () => { setPlaying(false); setFlowMode(false); setDeployState(1); setFlying(true); };
  const stopFly = () => { setFlying(false); inputRef.current = { roll: 0, pitch: 0 }; };

  const live = useMemo(() => {
    if (!telem) return null;
    if (deployState >= 0.99) return { V: telem.settle.V_glide, gamma: -Math.atan2(1, telem.settle.glide_ratio) * (180 / Math.PI), n: 1.0, ld: telem.settle.glide_ratio };
    const d = telem.series.deploy; let idx = 0; for (let i = 0; i < d.length; i++) { if (d[i] <= deployState) idx = i; else break; }
    return { V: telem.series.V[idx], gamma: telem.series.gamma_deg[idx], n: telem.series.n_load[idx], ld: telem.series.glide_ratio[idx] };
  }, [telem, deployState]);

  const btn = "rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition";
  const ld = hud.V > 0 ? (hud.V * Math.cos(hud.gamma)) / Math.max(-hud.V * Math.sin(hud.gamma), 0.05) : 0;
  const hold = (key: string) => ({ onPointerDown: () => (keys.current[key] = true), onPointerUp: () => (keys.current[key] = false), onPointerLeave: () => (keys.current[key] = false), onTouchStart: () => (keys.current[key] = true), onTouchEnd: () => (keys.current[key] = false) });

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas shadows dpr={[1, 2]} camera={{ position: [4.2, 2.4, 5.2], fov: 38, near: 0.05, far: 80 }}
        onCreated={({ scene }) => { scene.background = new THREE.Color("#070809"); }}>
        <Scene flyRef={flyRef} flying={flying} flightRef={flightRef} inputRef={inputRef} control={control} deployRef={deployRef} flowMode={flowMode} />
      </Canvas>

      <div className="absolute top-3 right-3 flex flex-col gap-2 max-w-[220px]">
        <button onClick={() => { stopFly(); setFlowMode(false); setPlaying(true); }} disabled={playing}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 disabled:opacity-40 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition">
          {playing ? "Deploying…" : "▶ Play deployment"}
        </button>
        <button onClick={() => (flying ? stopFly() : startFly())}
          className={flying ? "rounded-md bg-emerald-500/90 hover:bg-emerald-500 border border-emerald-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition" : btn}>
          {flying ? "✓ Flying — land/exit" : "🕹 Fly it"}
        </button>
        <button onClick={() => (flowMode ? setFlowMode(false) : (stopFly(), setPlaying(false), setDeployState(1), setFlowMode(true)))}
          className={flowMode ? "rounded-md bg-sky-500/90 hover:bg-sky-500 border border-sky-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition" : btn}>
          {flowMode ? "✓ Flow field ON" : "≈ Flow field"}
        </button>
        {!flying && <>
          <button onClick={() => { setFlowMode(false); setPlaying(false); setDeployState(0); }} className={btn}>⤴ Stowed (0%)</button>
          <button onClick={() => { setPlaying(false); setDeployState(1); }} className={btn}>⤵ Deployed (100%)</button>
        </>}
      </div>

      {flying && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/70 border border-emerald-900/60 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm min-w-[172px]">
          <div className="text-emerald-400/80 uppercase tracking-wider text-[10px]">you are flying it</div>
          <Telem label="airspeed" value={`${hud.V.toFixed(1)} m/s`} />
          <Telem label="altitude" value={`${Math.max(0, ALT0 + hud.h).toFixed(0)} m`} />
          <Telem label="bank φ" value={`${r2d(hud.phi).toFixed(0)}°`} />
          <Telem label="load factor" value={`${hud.n.toFixed(2)} g`} />
          <Telem label="α (AoA)" value={`${r2d(hud.alpha).toFixed(1)}°${hud.limited ? "  ⚠ LIMIT" : ""}`} />
          <Telem label="vario" value={`${(hud.V * Math.sin(hud.gamma)).toFixed(1)} m/s`} />
          <Telem label="glide L/D" value={ld > 0 && ld < 40 ? `${ld.toFixed(1)} : 1` : "—"} />
          <div className="mt-1 pt-1 border-t border-zinc-800 text-[10px] text-zinc-500 leading-snug">← → / A D roll · ↑ ↓ / W S pitch. α-limiter on — can't stall it.</div>
        </div>
      )}
      {live && !flowMode && !flying && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/70 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm">
          <div className="text-zinc-500 uppercase tracking-wider text-[10px]">flight physics · live</div>
          <Telem label="airspeed" value={`${live.V.toFixed(1)} m/s`} />
          <Telem label="flight path γ" value={`${live.gamma.toFixed(1)}°`} />
          <Telem label="load factor" value={`${live.n.toFixed(2)} g`} />
          <Telem label="glide ratio" value={live.ld > 0 && live.ld < 40 ? `${live.ld.toFixed(1)} : 1` : "—"} />
        </div>
      )}
      {flowMode && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/75 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm max-w-[230px]">
          <div className="text-zinc-400 uppercase tracking-wider text-[10px]">surface pressure · Cp</div>
          <div className="h-2 w-full rounded" style={{ background: "linear-gradient(90deg,#1a33d9,#19bff2,#34d94d,#fad926,#f22620)" }} />
          <div className="flex justify-between text-[10px] text-zinc-500"><span>suction (lift)</span><span>stagnation</span></div>
          <div className="text-[10px] text-zinc-500 leading-snug pt-1">Weissinger span-load + thin-airfoil Cp — illustrative, not CFD.</div>
        </div>
      )}

      {flying && (
        <div className="absolute bottom-3 right-3 grid grid-cols-3 gap-1 select-none" style={{ touchAction: "none" }}>
          <span /><button className={btn} {...hold("arrowup")}>▲</button><span />
          <button className={btn} {...hold("arrowleft")}>◀</button>
          <button className={btn} {...hold("arrowdown")}>▼</button>
          <button className={btn} {...hold("arrowright")}>▶</button>
        </div>
      )}
      {!flying && (
        <div className="absolute bottom-3 left-3 right-3 flex items-center gap-3 bg-zinc-900/85 border border-zinc-800 px-3 py-2 rounded-md text-xs">
          <span className="text-zinc-500 whitespace-nowrap">deploy</span>
          <input type="range" min={0} max={1} step={0.005} value={deployState} disabled={flowMode}
            onChange={(e) => { setDeployState(parseFloat(e.target.value)); setPlaying(false); }}
            className="flex-1 accent-orange-500 disabled:opacity-50" aria-label="deployment state" />
          <span className="mono text-zinc-300 tabular-nums w-10 text-right">{(deployState * 100).toFixed(0)}%</span>
        </div>
      )}
      <div className="absolute bottom-[3.4rem] left-3 text-[10px] text-zinc-600">
        S 6.5 m² · b 6.3 m · AR 6.1 · double-surface wing · deploy + flyable flight model
      </div>
    </div>
  );
}

function Telem({ label, value }: { label: string; value: string }) {
  return (<div className="flex items-center justify-between gap-4"><span className="text-zinc-500">{label}</span>
    <span className="mono text-zinc-200 tabular-nums">{value}</span></div>);
}
