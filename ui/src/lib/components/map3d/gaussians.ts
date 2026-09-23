import * as THREE from 'three';

/** SWGS v1: 16-byte header then float32 xyz, scales, quaternion xyzw, linear RGB, opacity.
 * Renders projected anisotropic covariance, depth-sorted alpha blending, DC color only.
 */
export class GaussianLayer {
  public group = new THREE.Group();
  public count = 0;
  public bounds = new THREE.Box3();
  /**
   * Called when the splats on screen changed outside a render: a depth sort
   * came back from the worker, or the model was cleared. The map draws on
   * demand, so without this the canvas keeps the old picture.
   */
  public onDirty: () => void = () => {};
  private mesh: THREE.Mesh<THREE.InstancedBufferGeometry, THREE.ShaderMaterial> | null = null;
  private records = new Float32Array();
  private worker: Worker;
  private generation = 0;
  private busy = false;
  private lastView = '';
  private lastSort = 0;
  private sortRetry: ReturnType<typeof setTimeout> | null = null;
  constructor() {
    this.worker = new Worker(new URL('./gaussian.worker.ts', import.meta.url), { type: 'module' });
    this.worker.onmessage = (e: MessageEvent<{ id: number; order: Uint32Array }>) => {
      if (e.data.id !== this.generation || !this.mesh) return;
      this.busy = false;
      const g = this.mesh.geometry;
      for (const [name, offset, width] of [
        ['center', 0, 3],
        ['scales', 3, 3],
        ['quaternion', 6, 4],
        ['tint', 10, 4]
      ] as const) {
        const a = g.getAttribute(name) as THREE.InstancedBufferAttribute;
        for (let i = 0; i < this.count; i++)
          for (let k = 0; k < width; k++)
            a.array[i * width + k] = this.records[e.data.order[i] * 14 + offset + k];
        a.needsUpdate = true;
      }
      this.onDirty();
    };
  }
  public load(buffer: ArrayBuffer, budget: number) {
    const h = new DataView(buffer);
    if (buffer.byteLength < 16 || h.getUint32(0, true) !== 0x53475753 || h.getUint32(4, true) !== 1)
      throw new Error('Unsupported Gaussian map');
    const n = h.getUint32(8, true);
    if (n > 2000000 || buffer.byteLength !== 16 + n * 56)
      throw new Error('Invalid Gaussian map length');
    const raw = new Float32Array(buffer, 16);
    for (let i = 0; i < raw.length; i++)
      if (!Number.isFinite(raw[i])) throw new Error('Invalid Gaussian map values');
    for (let i = 0; i < n; i++) {
      if (
        [3, 4, 5].some((k) => raw[i * 14 + k] <= 0) ||
        raw[i * 14 + 13] < 0 ||
        raw[i * 14 + 13] > 1
      )
        throw new Error('Invalid Gaussian scale or opacity');
    }
    this.clear();
    this.count = Math.min(n, budget);
    this.records = new Float32Array(this.count * 14);
    const centers = new Float32Array(this.count * 3);
    for (let i = 0; i < this.count; i++) {
      const index = Math.floor((i * n) / this.count);
      this.records.set(raw.subarray(index * 14, index * 14 + 14), i * 14);
      centers.set(this.records.subarray(i * 14, i * 14 + 3), i * 3);
    }
    this.bounds.setFromBufferAttribute(new THREE.BufferAttribute(centers, 3));
    const g = new THREE.InstancedBufferGeometry();
    g.setAttribute(
      'position',
      new THREE.Float32BufferAttribute([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0], 3)
    );
    g.setIndex([0, 1, 2, 0, 2, 3]);
    g.instanceCount = this.count;
    for (const [name, offset, width] of [
      ['center', 0, 3],
      ['scales', 3, 3],
      ['quaternion', 6, 4],
      ['tint', 10, 4]
    ] as const) {
      const a = new Float32Array(this.count * width);
      for (let i = 0; i < this.count; i++)
        a.set(this.records.subarray(i * 14 + offset, i * 14 + offset + width), i * width);
      g.setAttribute(
        name,
        new THREE.InstancedBufferAttribute(a, width).setUsage(THREE.DynamicDrawUsage)
      );
    }
    const material = new THREE.ShaderMaterial({
      transparent: true,
      depthWrite: false,
      depthTest: true,
      side: THREE.DoubleSide,
      uniforms: { viewport: { value: new THREE.Vector2(1, 1) }, ceiling: { value: 10000 } },
      vertexShader: `
        attribute vec3 center, scales; attribute vec4 quaternion, tint;
        uniform vec2 viewport; uniform float ceiling;
        varying vec2 uvG; varying vec4 colorG;
        void main() {
          uvG=position.xy*3.0; colorG=tint;
          vec4 c=modelViewMatrix*vec4(center,1.0);
          if(c.z>=-0.2 || center.z>ceiling){gl_Position=vec4(2.,2.,2.,1.);return;}
          vec4 q=normalize(quaternion); float x=q.x,y=q.y,z=q.z,w=q.w;
          mat3 R=mat3(1.-2.*(y*y+z*z),2.*(x*y+z*w),2.*(x*z-y*w),
                      2.*(x*y-z*w),1.-2.*(x*x+z*z),2.*(y*z+x*w),
                      2.*(x*z+y*w),2.*(y*z-x*w),1.-2.*(x*x+y*y));
          mat3 A=mat3(modelViewMatrix)*R*mat3(scales.x,0.,0.,0.,scales.y,0.,0.,0.,scales.z);
          float fx=projectionMatrix[0][0]*viewport.x*.5, fy=projectionMatrix[1][1]*viewport.y*.5;
          vec3 jx=vec3(-fx/c.z,0.,fx*c.x/(c.z*c.z));
          vec3 jy=vec3(0.,-fy/c.z,fy*c.y/(c.z*c.z));
          vec3 a=transpose(A)*jx, b=transpose(A)*jy;
          float xx=dot(a,a)+.3, yy=dot(b,b)+.3, xy=dot(a,b);
          float mid=.5*(xx+yy), delta=sqrt(max(0.,.25*(xx-yy)*(xx-yy)+xy*xy));
          vec2 axis=abs(xy)>0.00001 ? normalize(vec2(xy,mid+delta-xx)) : (xx>=yy ? vec2(1.,0.) : vec2(0.,1.));
          vec2 offset=uvG.x*min(sqrt(mid+delta),128.)*axis+uvG.y*min(sqrt(max(.1,mid-delta)),128.)*vec2(-axis.y,axis.x);
          vec4 clip=projectionMatrix*c; clip.xy+=offset*2./viewport*clip.w; gl_Position=clip;
        }`,
      fragmentShader: `
        varying vec2 uvG; varying vec4 colorG;
        void main(){float r=dot(uvG,uvG);if(r>9.)discard;
          float alpha=min(.99,colorG.a*exp(-.5*r));if(alpha<.0039)discard;
          gl_FragColor=vec4(colorG.rgb,alpha);
          #include <tonemapping_fragment>
          #include <colorspace_fragment>
        }`
    });
    this.mesh = new THREE.Mesh(g, material);
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 1;
    this.group.add(this.mesh);
    this.worker.postMessage({ centers, id: this.generation }, [centers.buffer]);
  }
  public update(camera: THREE.Camera, renderer: THREE.WebGLRenderer, ceiling: number) {
    if (!this.mesh || !this.group.visible) return;
    renderer.getDrawingBufferSize(this.mesh.material.uniforms.viewport.value);
    this.mesh.material.uniforms.ceiling.value = ceiling;
    const view = camera.matrixWorldInverse.elements;
    // Translation does not change the depth ordering.
    const signature = [view[2], view[6], view[10]].map((v) => v.toFixed(4)).join(',');
    if (this.busy || signature === this.lastView) return;
    const now = performance.now();
    const remaining = 100 - (now - this.lastSort);
    if (remaining > 0) {
      // On-demand rendering may stop at this viewpoint. Keep one wake-up so
      // it gets sorted even if neither camera nor telemetry changes again.
      if (this.sortRetry === null) {
        this.sortRetry = setTimeout(() => {
          this.sortRetry = null;
          this.onDirty();
        }, remaining);
      }
      return;
    }
    if (this.sortRetry !== null) clearTimeout(this.sortRetry);
    this.sortRetry = null;
    this.busy = true;
    this.lastView = signature;
    this.lastSort = now;
    this.worker.postMessage({ view, id: this.generation });
  }
  public clear() {
    if (this.sortRetry !== null) clearTimeout(this.sortRetry);
    this.sortRetry = null;
    this.bounds.makeEmpty();
    this.generation++;
    this.busy = false;
    this.lastView = '';
    this.count = 0;
    this.records = new Float32Array();
    if (this.mesh) {
      this.group.remove(this.mesh);
      this.mesh.geometry.dispose();
      this.mesh.material.dispose();
      this.mesh = null;
      this.onDirty();
    }
  }
  public dispose() {
    this.worker.terminate();
    this.clear();
  }
}
