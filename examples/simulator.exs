# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

Mix.install([
  {:ahrs, path: Path.expand("..", __DIR__)},
  {:phoenix_playground, "~> 0.1"}
])

defmodule Simulator.Live do
  use Phoenix.LiveView
  alias Ahrs.Quaternion, as: Q
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Math

  @tick_ms 20
  @dt 0.02

  def mount(_params, _session, socket) do
    if connected?(socket) do
      :timer.send_interval(@tick_ms, self(), :tick)
    end

    socket =
      socket
      |> assign(:true_q, %Q{})
      |> assign(:ahrs, Ahrs.new_madgwick())
      |> assign(:angular_velocity, {0.0, 0.0, 0.0})
      |> assign(:rotation_speed, 1.5)
      |> assign(:gyro_noise, 0.02)
      |> assign(:accel_noise, 0.05)
      |> assign(:filter_type, "madgwick")
      |> assign(:active_keys, MapSet.new())
      |> assign(:euler, {0.0, 0.0, 0.0})

    {:ok, socket}
  end

  def handle_info(:tick, socket) do
    {gx, gy, gz} = calculate_velocity(socket.assigns.active_keys, socket.assigns.rotation_speed)

    # 1. Integrate True orientation (perfect world)
    {dw, dx, dy, dz} = Q.gyro_derivative(socket.assigns.true_q, gx, gy, gz)

    true_q =
      %Q{
        w: socket.assigns.true_q.w + dw * @dt,
        x: socket.assigns.true_q.x + dx * @dt,
        y: socket.assigns.true_q.y + dy * @dt,
        z: socket.assigns.true_q.z + dz * @dt
      }
      |> Q.normalize()

    # 2. Generate Noisy Sensors
    gyro_noise = socket.assigns.gyro_noise
    accel_noise = socket.assigns.accel_noise

    gyro_reading = %Gyro{
      x: gx + random_noise(gyro_noise),
      y: gy + random_noise(gyro_noise),
      z: gz + random_noise(gyro_noise),
      units: :rad_s
    }

    {ax, ay, az} = Math.rotate_vector({0, 0, 1.0}, Q.conjugate(true_q))
    accel_reading = %Accel{
      x: ax + random_noise(accel_noise),
      y: ay + random_noise(accel_noise),
      z: az + random_noise(accel_noise),
      units: :g
    }

    # 3. Update Filter
    ahrs = Ahrs.update(socket.assigns.ahrs, %{accel: accel_reading, gyro: gyro_reading}, dt: @dt)

    # 4. Broadcast orientation to JS
    filter_q = Ahrs.quaternion(ahrs)
    {roll, pitch, yaw} = Ahrs.euler_angles(ahrs, units: :degrees)

    socket =
      socket
      |> assign(:true_q, true_q)
      |> assign(:ahrs, ahrs)
      |> assign(:euler, {roll, pitch, yaw})
      |> push_event("update_orientation", Map.from_struct(filter_q))

    {:noreply, socket}
  end

  def handle_event("visualizer_ready", _params, socket) do
    IO.puts("[Simulator] Visualizer ready signal received. Pushing OBJ data...")
    {:noreply, push_event(socket, "load_obj", %{data: File.read!("examples/utah_teapot.obj")})}
  end

  def handle_event("keydown", %{"key" => key}, socket) do
    {:noreply, assign(socket, :active_keys, MapSet.put(socket.assigns.active_keys, key))}
  end

  def handle_event("keyup", %{"key" => key}, socket) do
    {:noreply, assign(socket, :active_keys, MapSet.delete(socket.assigns.active_keys, key))}
  end

  def handle_event("change_filter", %{"filter" => type}, socket) do
    ahrs = case type do
      "madgwick" -> Ahrs.new_madgwick()
      "mahony" -> Ahrs.new_mahony()
      "complementary" -> Ahrs.new_complementary()
    end
    {:noreply, assign(socket, ahrs: ahrs, filter_type: type)}
  end

  def handle_event("change_gyro_noise", %{"noise" => val}, socket) do
    {:noreply, assign(socket, gyro_noise: parse_float(val))}
  end

  def handle_event("change_accel_noise", %{"noise" => val}, socket) do
    {:noreply, assign(socket, accel_noise: parse_float(val))}
  end

  def handle_event("change_speed", %{"speed" => val}, socket) do
    {:noreply, assign(socket, rotation_speed: parse_float(val))}
  end

  defp parse_float(val) do
    case Float.parse(val) do
      {f, _} -> f
      :error -> 0.0
    end
  end

  defp random_noise(level), do: (:rand.uniform() - 0.5) * level

  defp calculate_velocity(keys, speed) do
    gx = (if MapSet.member?(keys, "a"), do: -speed, else: 0.0) + (if MapSet.member?(keys, "d"), do: speed, else: 0.0)
    gy = (if MapSet.member?(keys, "w"), do: speed, else: 0.0) + (if MapSet.member?(keys, "s"), do: -speed, else: 0.0)
    gz = (if MapSet.member?(keys, "q"), do: -speed, else: 0.0) + (if MapSet.member?(keys, "e"), do: speed, else: 0.0)
    {gx, gy, gz}
  end

  def render(assigns) do
    ~H"""
    <script src="https://cdn.tailwindcss.com"></script>
    <div class="relative w-full h-screen font-sans bg-black overflow-hidden">
      <!-- Visualization -->
      <div id="visualizer-container" class="absolute inset-0 z-0" phx-update="ignore">
        <div id="visualizer" phx-hook="ThreeJsView" class="w-full h-full"></div>
      </div>

      <!-- Controls Overlay -->
      <div class="absolute top-8 left-8 z-10 w-80 max-h-[calc(100vh-4rem)] space-y-4 bg-gray-800/80 backdrop-blur-md p-6 rounded-2xl shadow-2xl border border-white/10 text-white overflow-y-auto no-scrollbar">
        <h1 class="text-2xl font-black text-blue-500 tracking-tight border-b border-white/10 pb-2">AHRS Simulator</h1>
        
        <div class="space-y-1">
          <label class="block text-[10px] font-bold text-gray-400 uppercase tracking-widest">Algorithm</label>
          <form phx-change="change_filter">
            <select name="filter" class="w-full bg-gray-700/50 border-none rounded-lg p-2 text-sm font-medium focus:ring-2 focus:ring-blue-500 appearance-none cursor-pointer text-white">
              <option value="madgwick" selected={@filter_type == "madgwick"}>Madgwick (GD)</option>
              <option value="mahony" selected={@filter_type == "mahony"}>Mahony (PI)</option>
              <option value="complementary" selected={@filter_type == "complementary"}>Complementary</option>
            </select>
          </form>
        </div>

        <div class="space-y-1">
          <label class="block text-[10px] font-bold text-gray-400 uppercase tracking-widest flex justify-between">
            Rotation Speed <span><%= @rotation_speed %> rad/s</span>
          </label>
          <form phx-change="change_speed">
            <input type="range" name="speed" min="0.1" max="5.0" step="0.1" value={@rotation_speed} class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
          </form>
        </div>

        <div class="space-y-3 pt-2 border-t border-white/5">
          <h2 class="text-[10px] font-black text-gray-400 uppercase tracking-widest">Sensor Noise</h2>
          
          <div class="space-y-1">
            <label class="block text-[9px] font-bold text-gray-500 uppercase flex justify-between">
              Gyro (rad/s) <span><%= @gyro_noise %></span>
            </label>
            <form phx-change="change_gyro_noise">
              <input type="range" name="noise" min="0" max="0.5" step="0.005" value={@gyro_noise} class="w-full h-1.5 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
            </form>
          </div>

          <div class="space-y-1">
            <label class="block text-[9px] font-bold text-gray-500 uppercase flex justify-between">
              Accel (G) <span><%= @accel_noise %></span>
            </label>
            <form phx-change="change_accel_noise">
              <input type="range" name="noise" min="0" max="0.5" step="0.005" value={@accel_noise} class="w-full h-1.5 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-blue-500" />
            </form>
          </div>
        </div>

        <div class="bg-black/30 p-4 rounded-xl border border-white/5 space-y-3">
          <h2 class="text-[10px] font-black text-blue-400 uppercase tracking-wider">Orientation</h2>
          <% {r, p, y} = @euler %>
          <div class="grid grid-cols-3 gap-2 text-center">
            <div class="flex flex-col">
              <span class="text-[9px] text-gray-500 uppercase font-bold">Roll</span>
              <span class="font-mono text-sm text-blue-100"><%= Float.round(r, 1) %>°</span>
            </div>
            <div class="flex flex-col border-x border-white/5">
              <span class="text-[9px] text-gray-500 uppercase font-bold">Pitch</span>
              <span class="font-mono text-sm text-blue-100"><%= Float.round(p, 1) %>°</span>
            </div>
            <div class="flex flex-col">
              <span class="text-[9px] text-gray-500 uppercase font-bold">Yaw</span>
              <span class="font-mono text-sm text-blue-100"><%= Float.round(y, 1) %>°</span>
            </div>
          </div>
        </div>

        <div class="bg-blue-900/20 p-3 rounded-lg border border-blue-800/30 text-[10px]">
          <p class="font-bold text-blue-300 uppercase mb-1">Keys</p>
          <div class="grid grid-cols-3 gap-2 text-gray-400">
            <span>P: <b class="text-blue-400 font-mono">W/S</b></span>
            <span>R: <b class="text-blue-400 font-mono">A/D</b></span>
            <span>Y: <b class="text-blue-400 font-mono">Q/E</b></span>
          </div>
        </div>
      </div>

      <!-- Status Indicator -->
      <div class="absolute bottom-8 left-8 z-10 flex items-center gap-3 bg-gray-800/60 backdrop-blur-md px-4 py-2 rounded-full border border-white/10 shadow-lg">
        <div class="w-2 h-2 rounded-full bg-green-500 animate-pulse shadow-[0_0_8px_#22c55e]"></div>
        <span class="text-[9px] font-black text-gray-200 uppercase tracking-[0.2em]">Engine Connected</span>
      </div>
    </div>

    <style>
      .no-scrollbar::-webkit-scrollbar { display: none; }
      .no-scrollbar { -ms-overflow-style: none; scrollbar-width: none; }
    </style>

    <script type="importmap">
    {
      "imports": {
        "three": "https://cdnjs.cloudflare.com/ajax/libs/three.js/0.160.0/three.module.min.js",
        "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
      }
    }
    </script>
    <script type="module">
      import * as THREE from 'three';
      import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

      window.hooks = window.hooks || {};
      window.hooks.ThreeJsView = {
        mounted() {
          console.log("[Simulator] Hook mounted.");
          const container = this.el;
          const scene = new THREE.Scene();
          const camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 1000);
          const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
          
          renderer.setSize(container.clientWidth, container.clientHeight);
          renderer.setPixelRatio(window.devicePixelRatio);
          container.appendChild(renderer.domElement);

          const axes = new THREE.AxesHelper(5);
          scene.add(axes);

          scene.add(new THREE.AmbientLight(0xffffff, 1.0));
          const dirLight = new THREE.DirectionalLight(0xffffff, 2);
          dirLight.position.set(10, 10, 10);
          scene.add(dirLight);

          let model = null;
          const loader = new OBJLoader();

          this.pushEvent("visualizer_ready", {});

          this.handleEvent("load_obj", (payload) => {
            console.log("[Simulator] load_obj received.");
            try {
              if (model) scene.remove(model);
              model = loader.parse(payload.data);
              model.rotation.x = -Math.PI / 2;
              
              model.traverse((child) => {
                if (child.isMesh) {
                  child.material = new THREE.MeshPhongMaterial({ color: 0xffffff, shininess: 80, specular: 0x222222 });
                }
              });

              const box = new THREE.Box3().setFromObject(model);
              const size = box.getSize(new THREE.Vector3());
              const maxDim = Math.max(size.x, size.y, size.z);
              const scale = 5 / maxDim;
              model.scale.set(scale, scale, scale);

              scene.add(model);
            } catch (err) { console.error(err); }
          });

          camera.position.set(10, 10, 10);
          camera.lookAt(0, 0, 0);

          window.addEventListener("keydown", (e) => { if(!e.repeat) this.pushEvent("keydown", {key: e.key.toLowerCase()}); });
          window.addEventListener("keyup", (e) => { this.pushEvent("keyup", {key: e.key.toLowerCase()}); });

          this.handleEvent("update_orientation", (q) => {
            if (model) model.quaternion.set(q.x, q.y, q.z, q.w);
          });

          const animate = () => { requestAnimationFrame(animate); renderer.render(scene, camera); };
          window.addEventListener('resize', () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
          });
          animate();
        }
      };
    </script>
    """
  end
end

PhoenixPlayground.start(live: Simulator.Live)
