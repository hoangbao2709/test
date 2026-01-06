// app/lib/robotApi.ts
// Nếu NEXT_PUBLIC_API_BASE rỗng hoặc không set, dùng relative path (cùng domain)
const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE === undefined || process.env.NEXT_PUBLIC_API_BASE === ""
    ? "" // Relative path - API đi qua nginx proxy cùng domain
    : process.env.NEXT_PUBLIC_API_BASE;

export const DEFAULT_DOG_SERVER =
  process.env.NEXT_PUBLIC_DOGZILLA_BASE || "http://127.0.0.1:9000";

export const robotId = "robot-a";

// NEW: Helper to normalize robot base URL
export function normalizeRobotBase(url: string): string {
  const trimmed = url.trim();
  if (trimmed.startsWith("http://") || trimmed.startsWith("https://")) {
    return trimmed.replace(/\/+$/, "");
  }
  // Default to HTTPS for IPs (Caddy TLS)
  return `https://${trimmed.replace(/\/+$/, "")}`;
}

// NEW: Direct robot API (for LAN/local connections)
export const createRobotLocalAPI = (baseUrl: string) => {
  const base = normalizeRobotBase(baseUrl);
  
  return {
    status: async () => {
      const res = await fetch(`${base}/status`, { cache: "no-store" });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return res.json();
    },
    
    move: async (cmd: { vx: number; vy: number; vz: number; rx: number; ry: number; rz: number }) => {
      // Map Django API format to robot command format
      // Robot expects: {"command": "forward"} or {"command": "stop"}
      const { vx, vy, rz } = cmd;
      
      // Determine primary movement
      const mag = Math.max(Math.abs(vx), Math.abs(vy), Math.abs(rz));
      if (mag < 0.01) {
        // Stop
        return fetch(`${base}/control`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ command: "stop" }),
        }).then(r => r.ok ? { log: "stop" } : Promise.reject(new Error(`HTTP ${r.status}`)));
      }
      
      // Determine direction
      let command = "stop";
      if (Math.abs(vx) > Math.abs(vy) && Math.abs(vx) > Math.abs(rz)) {
        command = vx > 0 ? "forward" : "back";
      } else if (Math.abs(vy) > Math.abs(rz)) {
        command = vy > 0 ? "right" : "left";
      } else {
        command = rz > 0 ? "turnleft" : "turnright";
      }
      
      const res = await fetch(`${base}/control`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ command }),
      });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return { log: command };
    },
    
    getVideoFeed: () => `${base}/camera`,
    
    getLidarUrl: () => `${base}/lidar/`,
  };
};

// Wrapper chung cho backend API
async function api<T = any>(path: string, init?: RequestInit): Promise<T> {

  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers || {}),
    },
    cache: "no-store",
  });

  const json = await res.json().catch(() => ({}));

  if (!res.ok) {
    const msg = (json as any)?.error || `HTTP ${res.status}`;
    const err: any = new Error(msg);
    err.status = res.status;
    err.body = json;
    throw err;
  }

  return json as T;
}

// Prefix đúng với Django: control/api/robots/...
const CONTROL_PREFIX = "/control/api/robots";

// Backend API (for Railway/production connections)
export const RobotAPI = {
  connect: (addr: string) =>
    api<{ connected: boolean; error?: string; log?: string }>(
      `${CONTROL_PREFIX}/${robotId}/connect/`,
      {
        method: "POST",
        body: JSON.stringify({ addr }),
      }
    ),

  status: () => api<any>(`${CONTROL_PREFIX}/${robotId}/status/`),

  fpv: () =>
    api<{ stream_url: string | null }>(
      `${CONTROL_PREFIX}/${robotId}/fpv/`
    ),

  speed: (mode: "slow" | "normal" | "high") =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/speed/`, {
      method: "POST",
      body: JSON.stringify({ mode }),
    }),

  move: (cmd: {
    vx: number;
    vy: number;
    vz: number;
    rx: number;
    ry: number;
    rz: number;
  }) =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/move/`, {
      method: "POST",
      body: JSON.stringify(cmd),
    }),

  lidar: (action: "start" | "stop") =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/lidar/`, {
      method: "POST",
      body: JSON.stringify({ action }),
    }),

  posture: (name: string) =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/posture/`, {
      method: "POST",
      body: JSON.stringify({ name }),
    }),

  behavior: (name: string) =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/behavior/`, {
      method: "POST",
      body: JSON.stringify({ name }),
    }),

  body: (sl: {
    tx: number;
    ty: number;
    tz: number;
    rx: number;
    ry: number;
    rz: number;
  }) =>
    api<any>(`${CONTROL_PREFIX}/${robotId}/command/body_adjust/`, {
      method: "POST",
      body: JSON.stringify(sl),
    }),

  stabilizingMode: (action: "on" | "off" | "toggle") =>
    api<any>(
      `${CONTROL_PREFIX}/${robotId}/command/stabilizing_mode/`,
      {
        method: "POST",
        body: JSON.stringify({ action }),
      }
    ),
};
