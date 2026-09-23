"""Fuselages, nacelles and booms: forces from slender-body theory and
viscous cross flow.

- Potential part (Munk; Multhopp's fuselage method is this with the wing's
  flow field): the force per unit length is rho U d/dx(S_a v_c), with S_a the
  apparent-mass area of the section for the cross-flow direction and v_c the
  local cross-flow velocity - free stream, rotation, and the up- and downwash
  the lifting surfaces induce at the body (from the vortex lattice). A closed
  body gets no net lift from it, only the destabilising Munk moment, reduced
  ahead of the wing's upwash and behind its downwash as Multhopp describes.
- Viscous cross flow (Allen & Perkins, NACA TR 1048): each section is a
  cylinder in the cross flow, drag q_c eta C_dc times its width across the
  flow - the lift and moment a body makes at high angles, and in sideslip.
- Friction along the axis: turbulent skin friction on the wetted area times
  a fineness form factor (Raymer 12.30).

Sections inside a wing root (where the lattice's carry-through panels carry
the load) have no potential or cross-flow force, as in Multhopp.
"""
import numpy as np

from .section import skin_friction


class BodyAero:
    def __init__(self, aircraft, re_speed, nu=1.46e-5, mach=0.0, stations=60):
        self.parts = []
        for body in aircraft.bodies:
            for side in body.copies():
                xs = np.linspace(body.x[0], body.x[-1], stations + 1)
                w, top, bot, yc, n = body.section(xs)
                h = np.maximum(top - bot, 0.0)
                y = yc * side
                z = 0.5 * (top + bot)
                pts = np.column_stack([xs, y, z])
                # apparent-mass areas: a section moving vertically carries a
                # cylinder of air on its width, sideways on its height
                s_v = np.pi * w**2 / 4.0
                s_h = np.pi * h**2 / 4.0
                # cross-flow drag coefficient: circle 1.2, boxier sections up to ~1.8
                cdc = np.clip(1.2 + 0.2 * (n - 2.0), 1.0, 1.8)
                fine = body.fineness
                eta = float(np.clip(0.55 + 0.02 * fine, 0.55, 0.85))  # Allen & Perkins' finite-length factor
                inside = np.zeros(len(xs), bool)
                for s in aircraft.surfaces:
                    root = s.sections[0]
                    if abs(root.le[1] - np.interp(root.le[0], xs, y)) > 0.5 * np.interp(root.le[0], xs, w) + 1e-6:
                        continue
                    zc = np.interp(root.le[0], xs, z)
                    hc = np.interp(root.le[0], xs, h)
                    if abs(root.le[2] - zc) > 0.5 * hc + 0.05 * root.chord or s.kind in ("fin", "vtail"):
                        continue
                    inside |= (xs >= root.le[0]) & (xs <= root.le[0] + root.chord)
                # potential flow holds from the nose to where the flow separates:
                # x0 = 0.378 l + 0.527 x1, x1 where the section shrinks fastest (DATCOM 4.2.1.1)
                area = np.pi * w * h / 4.0
                x1 = xs[int(np.argmin(np.gradient(area, xs)))]
                x0 = xs[0] + 0.378 * body.length + 0.527 * (x1 - xs[0])
                potential = xs <= x0
                re = re_speed * body.length / nu
                ff = 1.0 + 60.0 / fine**3 + fine / 400.0 if body.kind not in ("nacelle", "pod") else 1.0 + 0.35 / fine
                wet = self._wetted(body)
                self.parts.append(dict(name=body.name, x=xs, pts=pts, w=w, h=h, s_v=s_v, s_h=s_h, cdc=cdc, eta=eta,
                                       inside=inside, potential=potential, x0=x0,
                                       friction=skin_friction(re, mach) * ff * wet, wet=wet))

    @staticmethod
    def _wetted(body):
        v, t, _ = body.skin(48, 32)
        a = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1)
        return float(a.sum()) / len(body.copies())

    def points(self):
        return np.vstack([p["pts"] for p in self.parts]) if self.parts else np.zeros((0, 3))

    def forces(self, v, w, ref, induced=None, rho=1.0):
        """Force and moment (design frame, about ref) for air velocity v and
        rotation w; induced: (n_points, 3) lattice-induced velocity at
        points() (the wing's up- and downwash), or None."""
        F = np.zeros(3)
        M = np.zeros(3)
        k = 0
        for p in self.parts:
            pts = p["pts"]
            n = len(pts)
            vel = v[None, :] - np.cross(w[None, :], pts - ref)
            if induced is not None:
                vel = vel + induced[k : k + n]
            k += n
            U = vel[:, 0]
            vy, vz = vel[:, 1], vel[:, 2]
            dx = np.gradient(p["x"])
            live = ~p["inside"]
            # potential (slender body): f = rho U d/dx(S_a v_c), up to the separation station
            fz = rho * U * np.gradient(p["s_v"] * vz, p["x"]) * p["potential"]
            fy = rho * U * np.gradient(p["s_h"] * vy, p["x"]) * p["potential"]
            # viscous cross flow
            vc = np.hypot(vy, vz)
            width = np.sqrt((p["w"] * vz) ** 2 + (p["h"] * vy) ** 2) / np.maximum(vc, 1e-12)
            dc = 0.5 * rho * vc * p["eta"] * p["cdc"] * width  # times the cross-flow velocity vector
            fy = fy + dc * vy
            fz = fz + dc * vz
            f = np.column_stack([np.zeros(n), fy, fz]) * (dx * live)[:, None]
            # friction along the axis, spread over the length
            fax = 0.5 * rho * U * np.abs(U) * p["friction"] / (p["x"][-1] - p["x"][0])
            f[:, 0] += fax * dx
            F += f.sum(axis=0)
            M += np.cross(pts - ref, f).sum(axis=0)
        return F, M
