<script lang="ts">
  /**
   * Merged 3D point cloud, drawn in raw WebGL2.
   *
   * Why not three.js. The plan called for it under "adopt, don't author", but
   * that principle is about not re-implementing SLAM, Nav2 or MediaMTX — a
   * points-only viewer with an orbit camera is ~200 lines with no library, and
   * this frontend has deliberately kept exactly two dependencies (pako and
   * lucide). A 600 KB 3D engine to draw GL_POINTS is the wrong trade. If the
   * view ever needs meshes, lighting or picking, revisit it.
   *
   * The cloud is fetched whole and slowly, mirroring the 2D map's transport: it
   * is large, changes gradually, and is not on the operator's critical path.
   */
  import { onMount } from 'svelte';
  import { inflate } from 'pako';
  import { fleet } from '$lib/stores/fleet.svelte';

  let { active = false }: { active?: boolean } = $props();

  let canvas = $state<HTMLCanvasElement | null>(null);
  let points = $state(0);
  let robots = $state<string[]>([]);
  let error = $state<string | null>(null);
  let flat = $state(false);

  // Orbit camera, in the map frame: the fleet drives on z=0 so orbiting a point
  // above the floor is the only view that makes sense without a scene graph.
  let yaw = $state(-0.7);
  let pitch = $state(0.9);
  let distance = $state(28);
  const target: [number, number, number] = [0, 0, 0.6];

  let gl: WebGL2RenderingContext | null = null;
  let program: WebGLProgram | null = null;
  let positionBuffer: WebGLBuffer | null = null;
  let colourBuffer: WebGLBuffer | null = null;
  let count = 0;
  let raf = 0;
  // Reactive: the cursor style is bound to it in the markup below.
  let dragging = $state(false);
  let last: { x: number; y: number } | null = null;

  const VERTEX = `#version 300 es
    in vec3 a_position;
    in vec3 a_colour;
    uniform mat4 u_viewProjection;
    out vec3 v_colour;
    void main() {
      gl_Position = u_viewProjection * vec4(a_position, 1.0);
      // Nearer points get a slightly larger sprite, which reads as depth
      // without needing lighting or normals.
      gl_PointSize = clamp(120.0 / gl_Position.w, 1.0, 4.0);
      v_colour = a_colour;
    }`;

  const FRAGMENT = `#version 300 es
    precision mediump float;
    in vec3 v_colour;
    out vec4 outColour;
    void main() {
      outColour = vec4(v_colour, 1.0);
    }`;

  function compile(context: WebGL2RenderingContext, type: number, source: string) {
    const shader = context.createShader(type)!;
    context.shaderSource(shader, source);
    context.compileShader(shader);
    if (!context.getShaderParameter(shader, context.COMPILE_STATUS)) {
      throw new Error(context.getShaderInfoLog(shader) ?? 'shader compile failed');
    }
    return shader;
  }

  /** Column-major view-projection for an orbiting camera. */
  function viewProjection(aspect: number): Float32Array {
    const cp = Math.cos(pitch);
    const eye = [
      target[0] + distance * cp * Math.cos(yaw),
      target[1] + distance * cp * Math.sin(yaw),
      target[2] + distance * Math.sin(pitch)
    ];
    const f = normalise([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    const s = normalise(cross(f, [0, 0, 1]));
    const u = cross(s, f);

    const near = 0.5;
    const far = 300;
    const fov = 1.0;
    const t = 1 / Math.tan(fov / 2);
    const view = [
      s[0], u[0], -f[0], 0,
      s[1], u[1], -f[1], 0,
      s[2], u[2], -f[2], 0,
      -dot(s, eye), -dot(u, eye), dot(f, eye), 1
    ];
    const proj = [
      t / aspect, 0, 0, 0,
      0, t, 0, 0,
      0, 0, (far + near) / (near - far), -1,
      0, 0, (2 * far * near) / (near - far), 0
    ];
    return new Float32Array(multiply(proj, view));
  }

  const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a: number[], b: number[]) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
  function normalise(v: number[]) {
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / n, v[1] / n, v[2] / n];
  }
  function multiply(a: number[], b: number[]) {
    const out = new Array(16).fill(0);
    for (let c = 0; c < 4; c++)
      for (let r = 0; r < 4; r++)
        for (let k = 0; k < 4; k++) out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
    return out;
  }

  function hexToRgb(hex: string): [number, number, number] {
    const m = /^#?([\da-f]{2})([\da-f]{2})([\da-f]{2})$/i.exec(hex);
    if (!m) return [0.4, 0.5, 0.6];
    return [parseInt(m[1], 16) / 255, parseInt(m[2], 16) / 255, parseInt(m[3], 16) / 255];
  }

  async function fetchCloud() {
    if (!gl || !positionBuffer || !colourBuffer) return;
    try {
      const response = await fetch('/api/map/cloud', { cache: 'no-store' });
      if (!response.ok) throw new Error(`cloud ${response.status}`);
      const total = Number(response.headers.get('X-Cloud-Points') ?? 0);
      const scale = Number(response.headers.get('X-Cloud-Scale') ?? 0.01);
      const names = (response.headers.get('X-Cloud-Robots') ?? '')
        .split(',')
        .filter(Boolean);
      const raw = inflate(new Uint8Array(await response.arrayBuffer()));

      // int16 xyz triples, then one uint8 robot index per point.
      const xyz = new Int16Array(raw.buffer, raw.byteOffset, total * 3);
      const owners = new Uint8Array(raw.buffer, raw.byteOffset + total * 6, total);

      const positions = new Float32Array(total * 3);
      for (let i = 0; i < total * 3; i++) positions[i] = xyz[i] * scale;
      const palette = names.map((id) => hexToRgb(fleet.colorOf(id)));
      const colours = new Float32Array(total * 3);
      for (let i = 0; i < total; i++) {
        const c = palette[owners[i]] ?? [0.4, 0.5, 0.6];
        colours[i * 3] = c[0];
        colours[i * 3 + 1] = c[1];
        colours[i * 3 + 2] = c[2];
      }

      gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, colourBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, colours, gl.DYNAMIC_DRAW);
      count = total;
      points = total;
      robots = names;
      error = null;
      // A cloud with no height is not a rendering fault, it is the backend
      // sending a ground projection — RTAB-Map's `cloud_map` is 2D unless
      // `grid_3d:=true`. Say so, because a flat plane in a 3D view reads as a
      // broken viewer and sends you looking in the wrong place. Measured on a
      // real cloud, not assumed: check the actual z spread.
      let lo = Infinity;
      let hi = -Infinity;
      for (let i = 2; i < positions.length; i += 3) {
        if (positions[i] < lo) lo = positions[i];
        if (positions[i] > hi) hi = positions[i];
      }
      flat = total > 0 && hi - lo < 0.05;
    } catch (e) {
      error = e instanceof Error ? e.message : String(e);
    }
  }

  function render() {
    raf = requestAnimationFrame(render);
    if (!gl || !canvas || !program) return;
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.floor(canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(canvas.clientHeight * dpr));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.96, 0.96, 0.97, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    if (!count) return;
    gl.useProgram(program);
    gl.uniformMatrix4fv(
      gl.getUniformLocation(program, 'u_viewProjection'),
      false,
      viewProjection(w / h)
    );
    gl.drawArrays(gl.POINTS, 0, count);
  }

  onMount(() => {
    if (!canvas) return;
    // `preserveDrawingBuffer` so the canvas can be screenshotted. Without it the
    // drawing buffer is undefined after compositing, and any ReadPixels from
    // outside a frame — which is exactly how headless Chrome captures a page —
    // comes back empty. The view looks fine interactively and blank in every
    // screenshot, which is a miserable thing to debug.
    const context = canvas.getContext('webgl2', {
      antialias: true,
      preserveDrawingBuffer: true
    });
    if (!context) {
      error = 'WebGL2 unavailable in this browser';
      return;
    }
    gl = context;
    try {
      program = gl.createProgram()!;
      gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, VERTEX));
      gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, FRAGMENT));
      gl.linkProgram(program);
      if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        throw new Error(gl.getProgramInfoLog(program) ?? 'link failed');
      }
    } catch (e) {
      error = e instanceof Error ? e.message : String(e);
      return;
    }
    gl.enable(gl.DEPTH_TEST);

    positionBuffer = gl.createBuffer();
    colourBuffer = gl.createBuffer();
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    for (const [name, buffer] of [
      ['a_position', positionBuffer],
      ['a_colour', colourBuffer]
    ] as const) {
      const location = gl.getAttribLocation(program, name);
      gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
      gl.enableVertexAttribArray(location);
      gl.vertexAttribPointer(location, 3, gl.FLOAT, false, 0, 0);
    }

    void fetchCloud();
    // Slow: the merged cloud is an accumulated map, not a sensor stream.
    const poll = window.setInterval(() => active && void fetchCloud(), 5000);
    raf = requestAnimationFrame(render);
    return () => {
      window.clearInterval(poll);
      cancelAnimationFrame(raf);
    };
  });

  function onPointerDown(e: PointerEvent) {
    dragging = true;
    last = { x: e.clientX, y: e.clientY };
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
  }
  function onPointerMove(e: PointerEvent) {
    if (!dragging || !last) return;
    yaw -= (e.clientX - last.x) * 0.006;
    pitch = Math.max(0.05, Math.min(1.5, pitch + (e.clientY - last.y) * 0.006));
    last = { x: e.clientX, y: e.clientY };
  }
  function onPointerUp() {
    dragging = false;
    last = null;
  }
  function onWheel(e: WheelEvent) {
    e.preventDefault();
    distance = Math.max(3, Math.min(120, distance * (e.deltaY > 0 ? 1.1 : 0.9)));
  }
</script>

<div class="relative h-full w-full">
  <canvas
    bind:this={canvas}
    class="h-full w-full touch-none {dragging ? 'cursor-grabbing' : 'cursor-grab'}"
    onpointerdown={onPointerDown}
    onpointermove={onPointerMove}
    onpointerup={onPointerUp}
    onpointercancel={onPointerUp}
    onwheel={onWheel}
  ></canvas>

  <div
    class="panel-glow pointer-events-none absolute left-3 top-12 z-20 max-w-md rounded-[4px] border border-border
           bg-surface/90 px-2 py-1 text-[10px] text-fg-dim backdrop-blur-xl"
  >
    {#if error}
      <span class="text-warn">3D unavailable · {error}</span>
    {:else if !points}
      <!-- Not an error: only a 3D SLAM backend produces a cloud. -->
      No cloud yet · needs a 3D SLAM backend
    {:else}
      {points.toLocaleString()} points · {robots.length} robot{robots.length === 1 ? '' : 's'}
      <span class="ml-1 text-fg-dim/70">drag to orbit · scroll to zoom</span>
      {#if flat}
        <div class="mt-0.5 text-warn">
          Cloud is flat — SLAM is publishing a ground projection. Relaunch with
          <code>grid_3d:=true</code> for real structure.
        </div>
      {/if}
    {/if}
  </div>
</div>
