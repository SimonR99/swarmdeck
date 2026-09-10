<script lang="ts">
  import { onMount } from 'svelte';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import { Database, ExternalLink, RefreshCw, X } from 'lucide-svelte';
  import { replicaTactical } from '$lib/stores/replicaTactical.svelte';
  import type { ReplicaView } from '../map3d/replicaTactical';
  import {
    MAX_CACHE_BYTES,
    MAX_PREVIEW_POINTS,
    ReplicaChunkCache,
    ReplicaRequestGate,
    parseXYZF32,
    samplePreviewChunks,
    selectPreviewRefs,
    type ChunkRef,
    type PreviewChunk
  } from './replicaPreview';

  type ReplicaIndex = { robot_id: string; session_id: string; revision: number };

  let {
    open = false,
    onclose = () => {}
  }: { open?: boolean; onclose?: () => void } = $props();

  let replicas = $state<ReplicaIndex[]>([]);
  let selectedKey = $state('');
  let selectedComponent = $state('');
  let view = $state<ReplicaView | null>(null);
  let loading = $state(false);
  let error = $state('');
  let viewport = $state<HTMLDivElement | null>(null);
  let pointCount = $state(0);
  let cacheVersion = $state(0);
  let previewHashes = new Set<string>();
  let partialPreview = $state(false);

  const chunkCache = new ReplicaChunkCache(MAX_CACHE_BYTES);
  const requestGate = new ReplicaRequestGate();
  const MAX_DOWNLOADS = 3;

  let renderer: THREE.WebGLRenderer | null = null;
  let scene: THREE.Scene | null = null;
  let camera: THREE.PerspectiveCamera | null = null;
  let controls: OrbitControls | null = null;
  let previewRoot: THREE.Group | null = null;
  let resizeObserver: ResizeObserver | null = null;
  let animationFrame = 0;
  let renderDirty = true;
  let cameraComponent = '';
  let geometrySignature = '';
  const submapGroups = new Map<string, THREE.Group>();

  function keyOf(replica: ReplicaIndex) {
    return `${replica.robot_id}\u0000${replica.session_id}`;
  }

  function selectedReplica() {
    return replicas.find((entry) => keyOf(entry) === selectedKey) ?? null;
  }

  function showInTacticalMap() {
    if (!view?.selected || !view.component_id) return;
    replicaTactical.show({
      robotId: view.robot_id,
      sessionId: view.session_id,
      componentId: view.component_id
    });
    onclose();
  }

  function isAbort(reason: unknown) {
    return reason instanceof DOMException && reason.name === 'AbortError';
  }

  async function fetchIndex(signal: AbortSignal, generation: number) {
    const response = await fetch('/api/autonomy/replicas', { cache: 'no-store', signal });
    if (!response.ok) throw new Error(`Replica index unavailable (${response.status})`);
    const body = await response.json();
    if (!Array.isArray(body.replicas)) throw new Error('Invalid replica index');
    if (!requestGate.isCurrent(generation)) return;
    replicas = body.replicas.filter(
      (entry: ReplicaIndex) => entry && typeof entry.robot_id === 'string' && typeof entry.session_id === 'string'
    );
    if (!replicas.some((entry) => keyOf(entry) === selectedKey)) {
      selectedKey = replicas.length ? keyOf(replicas[0]) : '';
      selectedComponent = '';
    }
  }

  async function fetchChunk(ref: ChunkRef, signal: AbortSignal) {
    const cached = chunkCache.get(ref.sha256);
    if (cached) return cached;
    const response = await fetch(`/api/autonomy/chunks/${encodeURIComponent(ref.sha256)}`, {
      cache: 'force-cache',
      signal
    });
    if (!response.ok) throw new Error(`Replica chunk unavailable (${response.status})`);
    const advertised = Number(response.headers.get('content-length') ?? 0);
    if (advertised > 8 * 1024 * 1024) throw new Error('Replica chunk exceeds the browser safety limit');
    const points = parseXYZF32(new Uint8Array(await response.arrayBuffer()));
    if (ref.size_bytes !== undefined && points.byteLength + 16 !== ref.size_bytes) {
      throw new Error('Replica chunk does not match its declared size');
    }
    chunkCache.set(ref.sha256, points);
    return points;
  }

  async function fetchChunks(refs: ChunkRef[], signal: AbortSignal) {
    const unique = [...new Map(refs.map((ref) => [ref.sha256, ref])).values()];
    let cursor = 0;
    async function worker() {
      while (cursor < unique.length) {
        const ref = unique[cursor++];
        await fetchChunk(ref, signal);
      }
    }
    await Promise.all(Array.from({ length: Math.min(MAX_DOWNLOADS, unique.length) }, () => worker()));
  }

  async function fetchView(replica: ReplicaIndex, component: string, generation: number, signal: AbortSignal) {
    const query = component ? `?component_id=${encodeURIComponent(component)}` : '';
    const response = await fetch(
      `/api/autonomy/replicas/view/${encodeURIComponent(replica.robot_id)}/${encodeURIComponent(replica.session_id)}${query}`,
      { cache: 'no-store', signal }
    );
    if (!response.ok) throw new Error(`Replica view unavailable (${response.status})`);
    const next = (await response.json()) as ReplicaView;
    if (!requestGate.isCurrent(generation)) return;
    const refs = next.chunks ?? [];
    const selected = selectPreviewRefs(refs);
    const hashes = new Set(selected.map((ref) => ref.sha256));
    chunkCache.retain(hashes);
    await fetchChunks(selected, signal);
    if (!requestGate.isCurrent(generation)) return;
    previewHashes = hashes;
    partialPreview = hashes.size < new Set(refs.map((ref) => ref.sha256)).size;
    view = next;
    pointCount = refs.reduce((total, ref) => total + (ref.point_count ?? 0), 0);
    cacheVersion += 1;
  }

  async function refresh(onlyChanged = false) {
    if (!open) return;
    const token = requestGate.begin();
    loading = true;
    error = '';
    try {
      await fetchIndex(token.signal, token.generation);
      if (!requestGate.isCurrent(token.generation)) return;
      const replica = selectedReplica();
      if (!replica) {
        view = null;
        pointCount = 0;
        return;
      }
      if (onlyChanged && view?.robot_id === replica.robot_id &&
          view.session_id === replica.session_id && view.revision === replica.revision) return;
      await fetchView(replica, selectedComponent, token.generation, token.signal);
    } catch (reason) {
      if (!isAbort(reason) && requestGate.isCurrent(token.generation)) {
        error = reason instanceof Error ? reason.message : 'Replica inspection failed';
      }
    } finally {
      if (requestGate.isCurrent(token.generation)) loading = false;
    }
  }

  function matrixFor(T: number[][]) {
    const matrix = new THREE.Matrix4();
    matrix.set(
      T[0][0], T[0][1], T[0][2], T[0][3],
      T[1][0], T[1][1], T[1][2], T[1][3],
      T[2][0], T[2][1], T[2][2], T[2][3],
      T[3][0], T[3][1], T[3][2], T[3][3]
    );
    return matrix;
  }

  function disposePreview() {
    for (const group of submapGroups.values()) {
      for (const child of group.children) {
        if (child instanceof THREE.Points) {
          child.geometry.dispose();
          if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose());
          else child.material.dispose();
        }
      }
    }
    submapGroups.clear();
    previewRoot?.clear();
    geometrySignature = '';
  }

  function fitCamera() {
    if (!previewRoot || !camera || !controls) return;
    previewRoot.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(previewRoot);
    if (bounds.isEmpty()) {
      camera.position.set(2, 2, 2);
      controls.target.set(0, 0, 0);
    } else {
      const center = bounds.getCenter(new THREE.Vector3());
      const radius = Math.max(bounds.getSize(new THREE.Vector3()).length() * 0.6, 0.5);
      camera.position.copy(center).add(new THREE.Vector3(radius, radius, radius));
      controls.target.copy(center);
    }
    camera.near = 0.01;
    camera.far = 100000;
    camera.updateProjectionMatrix();
    controls.update();
  }

  function updatePreview() {
    renderDirty = true;
    if (!previewRoot || !view?.selected) {
      disposePreview();
      return;
    }
    const chunks: PreviewChunk[] = [];
    for (const submap of view.selected.submaps) {
      for (const ref of submap.chunks) {
        if (!previewHashes.has(ref.sha256)) continue;
        const points = chunkCache.get(ref.sha256);
        if (points) chunks.push({ submapId: submap.submap_id, sha256: ref.sha256, points });
      }
    }
    const sampled = samplePreviewChunks(chunks);
    const nextSignature = `${view.component_id ?? ''}|${sampled.map((entry) => `${entry.submapId}:${entry.sha256}:${entry.points.length}`).join('|')}`;
    const poseOnly = nextSignature === geometrySignature;
    if (!poseOnly) {
      disposePreview();
      const bySubmap = new Map<string, THREE.Group>();
      for (const entry of sampled) {
        let group = bySubmap.get(entry.submapId);
        if (!group) {
          group = new THREE.Group();
          group.matrixAutoUpdate = false;
          bySubmap.set(entry.submapId, group);
          submapGroups.set(entry.submapId, group);
          previewRoot.add(group);
        }
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(entry.points, 3));
        const material = new THREE.PointsMaterial({ color: 0x67e8f9, size: 0.035, sizeAttenuation: true });
        group.add(new THREE.Points(geometry, material));
      }
      geometrySignature = nextSignature;
    }
    for (const submap of view.selected.submaps) {
      const group = submapGroups.get(submap.submap_id);
      if (group) {
        group.matrix.copy(matrixFor(submap.T_component_submap));
        group.matrixWorldNeedsUpdate = true;
      }
    }
    const component = `${view.robot_id}/${view.session_id}/${view.component_id}`;
    if (cameraComponent !== component) {
      fitCamera();
      cameraComponent = component;
    }
  }

  function resizePreview() {
    if (!renderer || !camera || !viewport) return;
    const width = Math.max(1, viewport.clientWidth);
    const height = Math.max(1, viewport.clientHeight);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderDirty = true;
  }

  function setupPreview() {
    if (!viewport || renderer) return;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);
    camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100000);
    camera.up.set(0, 0, 1);
    camera.position.set(2, 2, 2);
    renderer = new THREE.WebGLRenderer({ antialias: false, alpha: false, powerPreference: 'low-power' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    viewport.appendChild(renderer.domElement);
    previewRoot = new THREE.Group();
    scene.add(previewRoot);
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.screenSpacePanning = true;
    controls.addEventListener('change', () => { renderDirty = true; });
    resizeObserver = new ResizeObserver(resizePreview);
    resizeObserver.observe(viewport);
    resizePreview();
    const render = () => {
      if (!renderer || !scene || !camera || !controls) return;
      controls.enabled = open;
      controls.update();
      if (open && renderDirty) {
        renderer.render(scene, camera);
        renderDirty = false;
      }
      animationFrame = requestAnimationFrame(render);
    };
    animationFrame = requestAnimationFrame(render);
  }

  function teardownPreview() {
    if (!renderer) return;
    cancelAnimationFrame(animationFrame);
    resizeObserver?.disconnect();
    disposePreview();
    controls?.dispose();
    renderer.dispose();
    renderer.domElement.remove();
    renderer = null;
    scene = null;
    camera = null;
    controls = null;
    previewRoot = null;
    cameraComponent = '';
  }

  $effect(() => {
    if (viewport) setupPreview();
    else teardownPreview();
  });

  $effect(() => {
    void view;
    void cacheVersion;
    if (renderer) updatePreview();
  });

  $effect(() => {
    if (open) void refresh();
    else requestGate.cancel();
  });

  onMount(() => {
    const timer = setInterval(() => {
      if (open && !loading) void refresh(true);
    }, 2000);
    return () => {
      clearInterval(timer);
      requestGate.cancel();
      teardownPreview();
    };
  });
</script>

{#if open}
  <aside
    class="panel-glow absolute right-3 top-3 z-40 flex max-h-[calc(100%-24px)] w-[min(420px,calc(100%-24px))] flex-col overflow-hidden rounded-[--radius-panel] border border-border/80 bg-surface/97 shadow-xl backdrop-blur-xl"
    aria-label="Onboard map replicas"
  >
    <header class="flex items-center gap-2 border-b border-border/70 px-4 py-3">
      <Database class="h-4 w-4 text-accent" />
      <div class="min-w-0 flex-1">
        <h2 class="text-sm font-semibold text-fg">Onboard map replicas</h2>
        <p class="text-[10px] text-fg-dim">Read-only component inspection</p>
      </div>
      <button class="rounded-full p-1.5 text-fg-dim hover:bg-surface-2 hover:text-fg" aria-label="Close replica inspector" onclick={onclose}>
        <X class="h-4 w-4" />
      </button>
    </header>

    <div class="space-y-3 overflow-y-auto p-4">
      <div class="flex items-center gap-2">
        <select class="min-w-0 flex-1 rounded-[--radius-control] border border-border bg-surface-2 px-2 py-2 text-[11px]" bind:value={selectedKey} onchange={() => { selectedComponent = ''; void refresh(); }} aria-label="Replica session">
          <option value="">No replicas</option>
          {#each replicas as replica}
            <option value={keyOf(replica)}>{replica.robot_id} · {replica.session_id.slice(0, 8)} · r{replica.revision}</option>
          {/each}
        </select>
        <button class="rounded-[--radius-control] border border-border p-2 text-fg-dim hover:bg-surface-2" title="Refresh replicas" onclick={() => void refresh()}>
          <RefreshCw class="h-3.5 w-3.5 {loading ? 'animate-spin' : ''}" />
        </button>
      </div>

      {#if view && view.components.length > 1}
        <label class="block text-[10px] text-fg-dim">
          Component
          <select class="mt-1 w-full rounded-[--radius-control] border border-border bg-surface-2 px-2 py-2 text-[11px]" bind:value={selectedComponent} onchange={() => void refresh()}>
            <option value="">Choose a disconnected component</option>
            {#each view.components as component}
              <option value={component.component_id}>{component.component_id}</option>
            {/each}
          </select>
        </label>
      {/if}

      {#if error}
        <div class="rounded-[--radius-control] bg-red-50 px-3 py-2 text-[11px] text-red-700" role="alert">{error}</div>
      {:else if !replicas.length && !loading}
        <div class="rounded-[--radius-control] bg-surface-2 px-3 py-3 text-[11px] text-fg-dim">No onboard replicas have been published.</div>
      {:else if view}
        {#if view.selected}
          <button
            class="flex w-full items-center justify-center gap-2 rounded-[--radius-control] bg-accent px-3 py-2 text-[11px] font-semibold text-accent-fg hover:brightness-105"
            onclick={showInTacticalMap}
          >
            <ExternalLink class="h-3.5 w-3.5" />
            Open component in tactical map
          </button>
        {/if}
        <div class="grid grid-cols-2 gap-2 text-[10px]">
          <div class="rounded-[--radius-control] bg-surface-2 px-2 py-2"><span class="text-fg-dim">Revision</span><br /><b>r{view.revision}</b></div>
          <div class="rounded-[--radius-control] bg-surface-2 px-2 py-2"><span class="text-fg-dim">Source age</span><br /><b>{view.source_age_s === null ? 'unknown clock' : `${Math.round(view.source_age_s)}s`}</b></div>
          <div class="rounded-[--radius-control] bg-surface-2 px-2 py-2"><span class="text-fg-dim">Component</span><br /><b class="break-all">{view.component_id ?? 'choose one'}</b></div>
          <div class="rounded-[--radius-control] bg-surface-2 px-2 py-2"><span class="text-fg-dim">Geometry</span><br /><b>{pointCount.toLocaleString()} points</b></div>
        </div>
        <div class="overflow-hidden rounded-[--radius-control] border border-border bg-surface-3">
          <div bind:this={viewport} class="h-56 w-full" aria-label="Selected replica 3D point preview"></div>
        </div>
        <p class="text-[10px] leading-relaxed text-fg-dim">
          Showing at most {MAX_PREVIEW_POINTS.toLocaleString()} points in the selected component frame.
          {#if partialPreview}Large map: preview samples a subset of chunks within a 64 MiB download budget.{/if}
          Orbit to inspect; disconnected components are never combined.
        </p>
        <div class="rounded-[--radius-control] bg-surface-2 px-3 py-2 text-[10px] text-fg-dim">
          Reconstruction: <span class="font-medium text-fg">{view.reconstruction?.state ?? 'unavailable'}</span>
        </div>
      {/if}
    </div>
  </aside>
{/if}
