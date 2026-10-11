use super::*;

impl Player {
    pub(crate) fn move_head(&mut self, dt: f32, enabled: bool) {
        let dt = dt.clamp(0.0, 0.1);
        let a = self.vehicle.physics.accel;
        let omega = self
            .vehicle
            .rigid
            .as_ref()
            .map(|rb| rb.omega)
            .unwrap_or(Vec3::ZERO);
        let dw = omega - self.head_omega;
        self.head_omega = omega;
        if dt <= 0.0 {
            return;
        }
        let def = &self.vehicle.ty.def;
        let n = def.cameras_driver.len().max(1);
        let eye = def
            .cameras_driver
            .get((def.camera_std + self.cam_choice.0) % n)
            .map(|c| Vec3::new(c.pos[0], c.pos[1], c.pos[2]))
            .unwrap_or(Vec3::ZERO);
        let stab = (1.0 / (15.0 * dt)).min(1.0);
        let spring = |p: f32, v: f32, kick: f32, k: f32, c: f32| -> (f32, f32) {
            let v = v + kick + (-k * p - c * v) * stab * dt;
            (p + v * dt, v)
        };
        let kick = -(a.z - 9.81) * dt + dw.y * eye.x + dw.x * eye.y;
        let (mut p, mut v) = spring(
            self.head.z,
            self.head_vel.z,
            kick,
            3000.0 / 150.0,
            2000.0 / 150.0,
        );
        if p.abs() > 0.1 {
            p = p.clamp(-0.1, 0.1);
            v = 0.0;
        }
        self.head.z = p;
        self.head_vel.z = v;
        if enabled {
            let moving = self.vehicle.physics.speed.abs().min(1.0);
            let kx = (-a.x * dt + dw.y * eye.z - dw.z * eye.y) * moving;
            let ky = (-a.y * dt - dw.x * eye.z + dw.z * eye.x) * moving;
            let (px, vx) = spring(
                self.head.x,
                self.head_vel.x,
                kx,
                3000.0 / 100.0,
                2000.0 / 100.0,
            );
            let (py, vy) = spring(
                self.head.y,
                self.head_vel.y,
                ky,
                3000.0 / 100.0,
                2000.0 / 100.0,
            );
            self.head.x = px;
            self.head_vel.x = vx;
            self.head.y = py;
            self.head_vel.y = vy;
        } else {
            self.head.x = 0.0;
            self.head.y = 0.0;
            self.head_vel.x = 0.0;
            self.head_vel.y = 0.0;
        }
    }

    pub(crate) fn auto_clutch_bite(&mut self, throttle: f32) {
        let program = &self.vehicle.ty.program;
        let has_manual_gate = program.manual_gearbox();
        let reads_clutch = program.var("Clutch").is_some_and(|v| program.reads(v));
        if self.vehicle.host.auto_clutch < 0.5
            || !has_manual_gate
            || !reads_clutch
            || program.reads_sys(::legacy_script::SysVar::AutoClutch)
        {
            return;
        }
        let gear = self
            .vehicle
            .var("antrieb_getr_aktugang")
            .or_else(|| self.vehicle.var("antrieb_getr_gang"))
            .unwrap_or(0.0);
        let kmh = self.vehicle.physics.velocity_kmh().abs();
        if gear.abs() > 0.5 && kmh < 12.0 {
            let n = self.vehicle.var("engine_n").unwrap_or(2000.0);
            let bite = ((throttle - 0.05) / 0.45)
                .clamp(0.0, 1.0)
                .min(((n - 850.0) / 700.0).clamp(0.0, 1.0));
            let bite = bite * bite * (3.0 - 2.0 * bite);
            let want = (1.0 - bite) * (1.0 - kmh / 12.0);
            self.axes.clutch = self.axes.clutch.max(want);
        }
    }

    pub(crate) fn gate_gear(&self) -> Option<i32> {
        let program = &self.vehicle.ty.program;
        program.trigger("kw_s_1")?;
        let v = gate_gear_var(program).and_then(|v| self.vehicle.var(&v));
        Some(v.unwrap_or(0.0).round() as i32)
    }

    pub(crate) fn shift_gate_to(&mut self, to: i32) -> bool {
        let name = match to {
            0 => "kw_s_N".to_string(),
            -1 => "kw_s_R".to_string(),
            n => format!("kw_s_{n}"),
        };
        if to < -1 || self.vehicle.ty.program.trigger(&name).is_none() {
            return false;
        }
        self.vehicle.set_var("Clutch", 1.0);
        self.axes.clutch = 1.0;
        self.vehicle.trigger(&name);
        self.vehicle.trigger(&format!("{name}_off"));
        true
    }

    pub(crate) fn tick_auto_shift(&mut self, dt: f32, throttle: f32, brake: f32) {
        self.auto_shift_wait = (self.auto_shift_wait - dt).max(0.0);
        if !self.auto_shift || !self.vehicle.ty.program.manual_gearbox() {
            return;
        }
        let Some(cur) = self.gate_gear() else { return };
        let Some(n) = [
            "engine_n",
            "antrieb_eng_n",
            "engine_rpm",
            "motor_n",
            "motor_rpm",
        ]
            .iter()
            .find_map(|v| self.vehicle.var(v)) else {
            return;
        };
        let kmh = self.vehicle.physics.velocity_kmh().abs();
        if n > 300.0 && kmh < 1.0 && throttle < 0.02 && (cur == 0 || self.axes.clutch > 0.9) {
            self.auto_shift_idle = if self.auto_shift_idle > 0.0 {
                self.auto_shift_idle + (n - self.auto_shift_idle) * (dt / 2.0).min(1.0)
            } else {
                n
            };
        }
        if n < 300.0
            || self.auto_shift_wait > 0.0
            || (self.axes.clutch > 0.5 && cur != 0 && kmh >= 5.0)
        {
            return;
        }
        let idle = if self.auto_shift_idle > 300.0 {
            self.auto_shift_idle
        } else {
            700.0
        };
        let top = (1..=12)
            .take_while(|g| {
                self.vehicle
                    .ty
                    .program
                    .trigger(&format!("kw_s_{g}"))
                    .is_some()
            })
            .last()
            .unwrap_or(0);
        let to = auto_shift_gear(cur, top, n / idle, throttle, brake, kmh);
        if to != cur && self.shift_gate_to(to) {
            self.auto_shift_wait = 1.5;
        }
    }

    pub(crate) fn tick(
        &mut self,
        dt: f32,
        audio: Option<&::audio::AudioEngine>,
        inside: bool,
        listener_follows_bus: bool,
    ) {
        self.tick_startup(dt);
        self.tick_auto_drag(dt);
        self.axes.speed_kmh = self.vehicle.physics.velocity_kmh();
        self.axes.lock_curvature = self.vehicle.ty.def.inv_min_turn_radius;
        self.axes.update(dt);
        let a = self.analog;
        if a.throttle.is_some_and(|t| t > 0.05) {
            self.axes.brake = 0.0;
        }
        self.auto_clutch_bite(a.throttle.unwrap_or(0.0).max(self.axes.throttle));
        self.tick_auto_shift(
            dt,
            a.throttle.unwrap_or(0.0).max(self.axes.throttle),
            a.brake.unwrap_or(0.0).max(self.axes.brake),
        );
        self.vehicle.set_controls(::simulation::Controls {
            throttle: a
                .throttle
                .unwrap_or(self.axes.throttle)
                .max(self.axes.throttle),
            brake: a.brake.unwrap_or(self.axes.brake).max(self.axes.brake),
            clutch: a.clutch.unwrap_or(self.axes.clutch).max(self.axes.clutch),
            steering: match a.steering {
                Some(s) if s.abs() > 0.02 || self.axes.steering == 0.0 => s,
                _ => self.axes.steering,
            },
        });
        let lever = self.vehicle.var("lights_sw_blinker");
        self.vehicle.update(dt);
        if let Some(keep) = kept_indicator(
            self.blinker_cancel,
            lever,
            self.vehicle.var("lights_sw_blinker"),
        ) {
            self.vehicle.set_var("lights_sw_blinker", keep);
        }
        if let Some(path) = ::legacy_config::env::var_os("OMSI_SUSP_TRACE_WINDOW") {
            use std::io::Write;
            static TRACE: std::sync::Mutex<Option<(std::fs::File, f64)>> =
                std::sync::Mutex::new(None);
            let mut g = TRACE.lock().unwrap_or_else(|e| e.into_inner());
            if g.is_none() {
                if let Ok(mut f) = std::fs::File::create(&path) {
                    let _ = writeln!(f, "t,dt,z,kmh,wheel,compression,rate,load,ground_z");
                    *g = Some((f, 0.0));
                }
            }
            if let (Some((f, t)), Some(rb)) = (g.as_mut(), self.vehicle.rigid.as_ref()) {
                *t += dt as f64;
                for (k, w) in rb.wheels.iter().enumerate() {
                    let _ = writeln!(
                        f,
                        "{:.4},{:.4},{:.4},{:.1},{k},{:.4},{:.3},{:.0},{:.4}",
                        t,
                        dt,
                        rb.position.z,
                        self.vehicle.physics.velocity_kmh(),
                        w.compression,
                        w.compression_rate,
                        w.load,
                        w.ground_z
                    );
                }
            }
        }
        self.tick_sounds(audio, inside, listener_follows_bus, true);
    }
}

pub(crate) fn keep_wheel(p: Option<&mut Player>) {
    if let Some(p) = p {
        p.axes.steering = p.vehicle.physics.controls.steering;
        p.axes.centering = false;
    }
}

pub(crate) fn mouse_steering(cursor_x: f32, width: f32, kmh: f32) -> f32 {
    let x = (2.0 * cursor_x / width.max(1.0) - 1.0).clamp(-1.0, 1.0);
    x / (kmh / 10.0).max(1.0)
}

pub(crate) fn mouse_pedal(current: f32, target: f32, k: f32) -> f32 {
    let v = target + (current - target) * k;
    if (v - target).abs() < 1e-4 { target } else { v }
}

#[cfg(test)]
mod mouse_tests {
    use super::{mouse_pedal, mouse_steering};

    #[test]
    fn the_mouse_pedal_reaches_the_floor() {
        for fps in [30.0f32, 60.0, 144.0] {
            let k = (-(1.0 / fps) / 0.06f32).exp();
            let (mut t, mut b) = (0.0, 1.0);
            for _ in 0..(fps as usize) {
                t = mouse_pedal(t, 1.0, k);
                b = mouse_pedal(b, 0.0, k);
            }
            assert_eq!(t, 1.0, "{fps} fps");
            assert_eq!(b, 0.0, "{fps} fps");
        }
        let k = (-(1.0 / 60.0f32) / 0.06).exp();
        let t = mouse_pedal(0.0, 1.0, k);
        assert!(t > 0.2 && t < 0.3, "{t}");
    }

    #[test]
    fn the_mouse_steers_less_the_faster_the_bus() {
        let w = 1600.0;
        assert_eq!(mouse_steering(0.0, w, 0.0), -1.0);
        assert_eq!(mouse_steering(1600.0, w, 5.0), 1.0);
        assert_eq!(mouse_steering(800.0, w, 0.0), 0.0);
        assert!((mouse_steering(1200.0, w, 10.0) - 0.5).abs() < 1e-6);
        assert!((mouse_steering(1200.0, w, 50.0) - 0.1).abs() < 1e-6);
        assert!((mouse_steering(1600.0, w, 100.0) - 0.1).abs() < 1e-6);
        assert!((mouse_steering(1200.0, w, -20.0) - 0.5).abs() < 1e-6);
        let mut last = mouse_steering(0.0, w, 30.0);
        for px in 1..=1600 {
            let s = mouse_steering(px as f32, w, 30.0);
            assert!((s - last).abs() < 0.001, "step at {px}");
            last = s;
        }
    }
}

pub(crate) fn auto_shift_gear(
    cur: i32,
    top: i32,
    n_over_idle: f32,
    throttle: f32,
    brake: f32,
    kmh: f32,
) -> i32 {
    if cur == 0 {
        return if throttle > 0.1 && brake < 0.05 && kmh < 3.0 && top >= 1 {
            1
        } else {
            0
        };
    }
    if cur < 1 {
        return cur;
    }
    if cur > 1 && kmh < 5.0 {
        return 1;
    }
    let up = 2.2 + 1.2 * throttle.clamp(0.0, 1.0);
    if cur < top && throttle > 0.05 && n_over_idle > up {
        cur + 1
    } else if cur > 1 && n_over_idle < 1.35 {
        cur - 1
    } else {
        cur
    }
}

#[cfg(test)]
mod auto_shift_tests {
    use super::auto_shift_gear as g;

    #[test]
    fn the_automated_manual_shifts_by_the_engine_speed() {
        assert_eq!(g(0, 5, 1.0, 0.5, 0.0, 0.0), 1);
        assert_eq!(g(0, 5, 1.0, 0.0, 0.0, 0.0), 0);
        assert_eq!(g(-1, 5, 3.0, 1.0, 0.0, 5.0), -1);
        assert_eq!(g(2, 5, 2.8, 0.3, 0.0, 30.0), 3);
        assert_eq!(g(2, 5, 2.8, 1.0, 0.0, 30.0), 2);
        assert_eq!(g(2, 5, 3.7, 1.0, 0.0, 30.0), 3);
        assert_eq!(g(5, 5, 4.0, 1.0, 0.0, 90.0), 5);
        assert_eq!(g(3, 5, 1.2, 0.0, 0.5, 20.0), 2);
        assert_eq!(g(1, 5, 1.0, 0.0, 1.0, 2.0), 1);
        assert_eq!(g(4, 5, 1.0, 0.0, 1.0, 3.0), 1);
    }
}
