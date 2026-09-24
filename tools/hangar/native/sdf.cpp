#include "sdf.h"

#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace meshkit {
namespace {

using fsim::core::Json;

[[noreturn]] void fail(const std::string& what) { throw std::runtime_error("meshkit: " + what); }

V3 vec(const Json& j) {
    if (!j.isArray() || j.asArray().size() != 3) fail("a vector needs three numbers");
    const auto& a = j.asArray();
    return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}

V3 vec(const Json& j, const char* key) {
    const Json* v = j.find(key);
    if (v == nullptr) fail(std::string("missing vector '") + key + "'");
    return vec(*v);
}

std::vector<double> list(const Json& j, const char* key) {
    const Json* v = j.find(key);
    if (v == nullptr || !v->isArray()) fail(std::string("missing list '") + key + "'");
    std::vector<double> out;
    out.reserve(v->asArray().size());
    for (const auto& e : v->asArray()) out.push_back(e.asNumber());
    return out;
}

int materialOf(const Json& j) { return static_cast<int>(j.number("material", 0.0)); }

/// Distance from (px, pz) to the segment (ax, az)-(bx, bz), squared.
double segment2(double px, double pz, double ax, double az, double bx, double bz) {
    const double ex = bx - ax, ez = bz - az;
    const double l2 = ex * ex + ez * ez;
    const double t = l2 > 0.0 ? clamp(((px - ax) * ex + (pz - az) * ez) / l2, 0.0, 1.0) : 0.0;
    const double dx = px - (ax + t * ex), dz = pz - (az + t * ez);
    return dx * dx + dz * dz;
}

double interp(const std::vector<double>& x, const std::vector<double>& y, double xq) {
    if (xq <= x.front()) return y.front();
    if (xq >= x.back()) return y.back();
    const auto it = std::upper_bound(x.begin(), x.end(), xq);
    const auto i = static_cast<std::size_t>(it - x.begin()) - 1;
    const double t = (xq - x[i]) / (x[i + 1] - x[i]);
    return lerp(y[i], y[i + 1], t);
}

// -- primitives ------------------------------------------------------------------------------

/// A body lofted along x through superelliptic sections: at each station the
/// widest line's centre (yc, zc), its half width hw, the height above it hu
/// and the depth below it hl, and an exponent for each half (2 an ellipse,
/// larger boxier, towards 1 a diamond: chines).
class Loft final : public Node {
public:
    explicit Loft(const Json& j)
        : x_(list(j, "x")), yc_(list(j, "yc")), zc_(list(j, "zc")), hw_(list(j, "hw")), hu_(list(j, "hu")),
          hl_(list(j, "hl")), nu_(list(j, "nu")), nl_(list(j, "nl")), mirror_(j.boolean("mirror", false)),
          mat_(materialOf(j)) {
        const std::size_t n = x_.size();
        if (n < 2) fail("a loft needs two stations");
        for (const auto* v : {&yc_, &zc_, &hw_, &hu_, &hl_, &nu_, &nl_})
            if (v->size() != n) fail("a loft's lists differ in length");
        for (std::size_t i = 1; i < n; ++i)
            if (x_[i] <= x_[i - 1]) fail("a loft's stations must increase in x");
        for (std::size_t i = 0; i < n; ++i) {
            const double y0 = yc_[i] - hw_[i], y1 = yc_[i] + hw_[i];
            box.add({x_[i], y0, zc_[i] - hl_[i]});
            box.add({x_[i], y1, zc_[i] + hu_[i]});
            if (mirror_) box.add({x_[i], -y1, zc_[i]});
            if (mirror_) box.add({x_[i], -y0, zc_[i]});
        }
    }

    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const double x0 = x_.front(), x1 = x_.back();
        const double xc = clamp(p.x, x0, x1);
        double d = dist(at(xc), p.y, p.z);
        // the section's distance ignores the body's slope along x: divide by
        // the gradient it leaves out
        const double h = 0.02;
        const double xa = std::max(x0, xc - h), xb = std::min(x1, xc + h);
        if (xb > xa) {
            const double s = (dist(at(xb), p.y, p.z) - dist(at(xa), p.y, p.z)) / (xb - xa);
            d /= std::sqrt(1.0 + s * s);
        }
        return {extrude(d, std::max(x0 - p.x, p.x - x1)), mat_};
    }

private:
    struct Sec {
        double yc, zc, hw, hu, hl, nu, nl;
    };

    Sec at(double x) const {
        const auto it = std::upper_bound(x_.begin(), x_.end(), x);
        std::size_t i = static_cast<std::size_t>(std::max<std::ptrdiff_t>(it - x_.begin(), 1)) - 1;
        i = std::min(i, x_.size() - 2);
        const double t = clamp((x - x_[i]) / (x_[i + 1] - x_[i]), 0.0, 1.0);
        return {lerp(yc_[i], yc_[i + 1], t), lerp(zc_[i], zc_[i + 1], t), lerp(hw_[i], hw_[i + 1], t),
                lerp(hu_[i], hu_[i + 1], t), lerp(hl_[i], hl_[i + 1], t), lerp(nu_[i], nu_[i + 1], t),
                lerp(nl_[i], nl_[i + 1], t)};
    }

    /// First-order distance to a superellipse half: (f - 1) / |grad f| with
    /// f = (|u|^n + |v|^n)^(1/n) - exact for circles, close near any surface.
    static double dist(const Sec& s, double y, double z) {
        const double dy = std::fabs(y - s.yc);
        const double rz = z - s.zc;
        const bool up = rz >= 0.0;
        const double dz = std::fabs(rz);
        const double a = std::max(s.hw, 1e-4), b = std::max(up ? s.hu : s.hl, 1e-4);
        const double n = std::max(up ? s.nu : s.nl, 1.0);
        const double u = dy / a, v = dz / b;
        if (u < 1e-12 && v < 1e-12) return -std::min(a, b);
        const double f = std::pow(std::pow(u, n) + std::pow(v, n), 1.0 / n);
        const double fp = std::pow(f, 1.0 - n);
        const double gy = u > 0.0 ? fp * std::pow(u, n - 1.0) / a : 0.0;
        const double gz = v > 0.0 ? fp * std::pow(v, n - 1.0) / b : 0.0;
        return (f - 1.0) / std::max(std::hypot(gy, gz), 1e-12);
    }

    std::vector<double> x_, yc_, zc_, hw_, hu_, hl_, nu_, nl_;
    bool mirror_;
    int mat_;
};

/// A frame along one segment of a lifting surface, shared by its solid and
/// by the regions its control surfaces are cut from: the leading edge from
/// le_a to le_b, chords and twists at both ends, and the span direction s
/// (in the y-z plane) the streamwise sections are normal to.
struct WingFrame {
    V3 leA, leB, span, c0{1.0, 0.0, 0.0}, u0;
    double chordA = 1.0, chordB = 1.0, twistA = 0.0, twistB = 0.0, length = 1.0;
    bool mirror = false;

    explicit WingFrame(const Json& j) {
        const Json& le = j.child("le");
        if (!le.isArray() || le.asArray().size() != 2) fail("a wing segment needs le = [a, b]");
        leA = vec(le.asArray()[0]);
        leB = vec(le.asArray()[1]);
        const auto ch = list(j, "chord");
        const auto tw = list(j, "twist");
        if (ch.size() != 2 || tw.size() != 2) fail("a wing segment needs two chords and two twists");
        chordA = ch[0];
        chordB = ch[1];
        twistA = tw[0];
        twistB = tw[1];
        span = normalize(vec(j, "span"));
        u0 = normalize(cross(c0, span));
        length = dot(leB - leA, span);
        mirror = j.boolean("mirror", false);
        if (length <= 1e-9 || chordA <= 0.0 || chordB <= 0.0) fail("a wing segment needs span and chord");
    }

    struct At {
        double t;      ///< along the segment, unclamped
        double chord;  ///< at the clamped station
        double xi, zeta; ///< chordwise and normal coordinates at the station
    };

    At locate(V3 p) const {
        if (mirror) p.y = std::fabs(p.y);
        const double t = dot(p - leA, span) / length;
        At a = at(p, clamp(t, 0.0, 1.0));
        a.t = t;
        return a;
    }

    /// p's coordinates in the section plane at station tc (p already mirrored).
    At at(V3 p, double tc) const {
        const double tw = lerp(twistA, twistB, tc);
        const V3 c = std::cos(tw) * c0 - std::sin(tw) * u0;
        const V3 u = std::sin(tw) * c0 + std::cos(tw) * u0;
        const V3 r = p - lerp(leA, leB, tc);
        return {tc, lerp(chordA, chordB, tc), dot(r, c), dot(r, u)};
    }

    Box bounds(double depth) const {
        Box b;
        for (int e = 0; e < 2; ++e) {
            const V3 le = e == 0 ? leA : leB;
            const double chord = e == 0 ? chordA : chordB;
            const double tw = e == 0 ? twistA : twistB;
            const V3 c = std::cos(tw) * c0 - std::sin(tw) * u0;
            const V3 u = std::sin(tw) * c0 + std::cos(tw) * u0;
            for (double f : {-0.05, 1.05})
                for (double h : {-depth, depth}) {
                    const V3 q = le + c * (f * chord) + u * (h * chord);
                    b.add(q);
                    if (mirror) b.add({q.x, -q.y, q.z});
                }
        }
        return b;
    }
};

/// One segment of a wing, tail, canard or fin: airfoil sections lofted
/// between two stations (tables blended along the span), inflated a little
/// so a sharp trailing edge keeps some thickness.
class Wing final : public Node {
public:
    Wing(const Json& j, const std::vector<std::unique_ptr<FoilTable>>& foils)
        : f_(j), inflate_(j.number("inflate", 0.0)), mat_(materialOf(j)) {
        const auto idx = list(j, "foils");
        if (idx.size() != 2) fail("a wing segment needs two foils");
        for (int e = 0; e < 2; ++e) {
            const auto k = static_cast<std::size_t>(idx[static_cast<std::size_t>(e)]);
            if (k >= foils.size()) fail("a wing segment names a foil that is not there");
            foil_[e] = foils[k].get();
        }
        box = f_.bounds(0.3).padded(inflate_);
    }

    Sample eval(V3 p) const override {
        if (f_.mirror) p.y = std::fabs(p.y);
        const double t = dot(p - f_.leA, f_.span) / f_.length;
        const double tc = clamp(t, 0.0, 1.0);
        double d = section(p, tc);
        // the section's distance is taken in its own plane; beside a swept or
        // tapered edge the surface leans along the span, and the distance
        // across it is the in-plane one over its gradient
        const double dt = std::min(0.02 / f_.length, 0.25);
        const double ta = std::max(0.0, tc - dt), tb = std::min(1.0, tc + dt);
        if (tb > ta) {
            const double g = (section(p, tb) - section(p, ta)) / ((tb - ta) * f_.length);
            d /= std::sqrt(1.0 + g * g);
        }
        return {extrude(d, std::max(-t, t - 1.0) * f_.length), mat_};
    }

private:
    /// Distance from p to the section at station t, in that section's plane.
    double section(V3 p, double t) const {
        const auto a = f_.at(p, t);
        const double X = a.xi / a.chord, Z = a.zeta / a.chord;
        return lerp(foil_[0]->sdf(X, Z), foil_[1]->sdf(X, Z), t) * a.chord - inflate_;
    }

    WingFrame f_;
    const FoilTable* foil_[2] = {nullptr, nullptr};
    double inflate_;
    int mat_;
};

/// The part of a wing segment between two stations (t0..t1 along it) and
/// two chord fractions (each varying linearly across the part): what a
/// control surface is cut from. Unbounded normal to the chord.
class WingRegion final : public Node {
public:
    explicit WingRegion(const Json& j) : f_(j) {
        const auto t = list(j, "t");
        const auto f0 = list(j, "front");
        const auto f1 = list(j, "back");
        if (t.size() != 2 || f0.size() != 2 || f1.size() != 2) fail("a wing region needs t, front and back pairs");
        t0_ = t[0];
        t1_ = t[1];
        front_[0] = f0[0];
        front_[1] = f0[1];
        back_[0] = f1[0];
        back_[1] = f1[1];
        box = f_.bounds(1.0).padded(1.0);
        mat_ = materialOf(j);
    }

    Sample eval(V3 p) const override {
        if (f_.mirror) p.y = std::fabs(p.y);
        const double t = dot(p - f_.leA, f_.span) / f_.length;
        const double tc = clamp(t, 0.0, 1.0);
        double dc = chordwise(p, tc);
        // as for the wing: across a swept or tapered front or back line the
        // distance is the in-plane one over its gradient along the span
        const double dt = std::min(0.02 / f_.length, 0.25);
        const double ta = std::max(0.0, tc - dt), tb = std::min(1.0, tc + dt);
        if (tb > ta) {
            const double g = (chordwise(p, tb) - chordwise(p, ta)) / ((tb - ta) * f_.length);
            dc /= std::sqrt(1.0 + g * g);
        }
        const double ds = std::max(t0_ - t, t - t1_) * f_.length;
        return {std::max(dc, ds), mat_};
    }

private:
    /// How far p lies outside the region's chordwise band at station t, in
    /// that section's plane (p already mirrored).
    double chordwise(V3 p, double t) const {
        const auto a = f_.at(p, t);
        const double s = t1_ > t0_ ? clamp((t - t0_) / (t1_ - t0_), 0.0, 1.0) : 0.0;
        const double front = lerp(front_[0], front_[1], s) * a.chord;
        const double back = lerp(back_[0], back_[1], s) * a.chord;
        return std::max(front - a.xi, a.xi - back);
    }

    WingFrame f_;
    double t0_ = 0.0, t1_ = 1.0, front_[2] = {0.0, 0.0}, back_[2] = {1.0, 1.0};
    int mat_ = 0;
};

/// A solid of revolution: a profile of radii r at stations s along an axis
/// from an origin, closed onto the axis at both ends.
class Revolve final : public Node {
public:
    explicit Revolve(const Json& j)
        : o_(vec(j, "origin")), ax_(normalize(vec(j, "axis"))), s_(list(j, "s")), r_(list(j, "r")),
          mirror_(j.boolean("mirror", false)), mat_(materialOf(j)) {
        if (s_.size() < 2 || r_.size() != s_.size()) fail("a revolve needs matching s and r");
        for (std::size_t i = 1; i < s_.size(); ++i)
            if (s_[i] < s_[i - 1]) fail("a revolve's stations must not decrease");
        double rmax = 0.0;
        for (double r : r_) rmax = std::max(rmax, r);
        for (double s : {s_.front(), s_.back()}) {
            const V3 c = o_ + ax_ * s;
            box.add(c - V3{rmax, rmax, rmax});
            box.add(c + V3{rmax, rmax, rmax});
            if (mirror_) {
                box.add({c.x - rmax, -c.y - rmax, c.z - rmax});
                box.add({c.x + rmax, -c.y + rmax, c.z + rmax});
            }
        }
    }

    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 rel = p - o_;
        const double s = dot(rel, ax_);
        const double rho = length(rel - ax_ * s);
        const std::size_t n = s_.size();
        double best = segment2(s, rho, s_[0], 0.0, s_[0], r_[0]);
        for (std::size_t i = 0; i + 1 < n; ++i) best = std::min(best, segment2(s, rho, s_[i], r_[i], s_[i + 1], r_[i + 1]));
        best = std::min(best, segment2(s, rho, s_[n - 1], r_[n - 1], s_[n - 1], 0.0));
        const bool inside = s > s_.front() && s < s_.back() && rho < interp(s_, r_, s);
        const double d = std::sqrt(best);
        return {inside ? -d : d, mat_};
    }

private:
    V3 o_, ax_;
    std::vector<double> s_, r_;
    bool mirror_;
    int mat_;
};

class Capsule final : public Node {
public:
    explicit Capsule(const Json& j)
        : a_(vec(j, "a")), b_(vec(j, "b")), r_(j.number("r", 0.05)), mirror_(j.boolean("mirror", false)),
          mat_(materialOf(j)) {
        for (V3 q : {a_, b_}) {
            box.add(q);
            if (mirror_) box.add({q.x, -q.y, q.z});
        }
        box = box.padded(r_);
    }
    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 pa = p - a_, ba = b_ - a_;
        const double bb = dot(ba, ba);
        const double h = bb > 0.0 ? clamp(dot(pa, ba) / bb, 0.0, 1.0) : 0.0;
        return {length(pa - ba * h) - r_, mat_};
    }

private:
    V3 a_, b_;
    double r_;
    bool mirror_;
    int mat_;
};

/// A capped cylinder from a to b, its rims rounded.
class Cylinder final : public Node {
public:
    explicit Cylinder(const Json& j)
        : a_(vec(j, "a")), b_(vec(j, "b")), r_(j.number("r", 0.05)), round_(j.number("round", 0.0)),
          mirror_(j.boolean("mirror", false)), mat_(materialOf(j)) {
        ax_ = normalize(b_ - a_);
        len_ = length(b_ - a_);
        for (V3 q : {a_, b_}) {
            box.add(q);
            if (mirror_) box.add({q.x, -q.y, q.z});
        }
        box = box.padded(r_);
    }
    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 rel = p - a_;
        const double t = dot(rel, ax_);
        const double rho = length(rel - ax_ * t);
        const double rr = round_;
        const double dr = rho - (r_ - rr);
        const double da = std::fabs(t - 0.5 * len_) - (0.5 * len_ - rr);
        return {extrude(dr, da) - rr, mat_};
    }

private:
    V3 a_, b_, ax_;
    double r_, round_, len_ = 0.0;
    bool mirror_;
    int mat_;
};

/// An oriented box with rounded edges: centre, three axes, half sizes.
class BoxPrim final : public Node {
public:
    explicit BoxPrim(const Json& j)
        : c_(vec(j, "centre")), h_(vec(j, "half")), round_(j.number("round", 0.0)),
          mirror_(j.boolean("mirror", false)), mat_(materialOf(j)) {
        const Json& axes = j.child("axes");
        if (axes.isArray() && axes.asArray().size() == 3) {
            for (std::size_t i = 0; i < 3; ++i) ax_[i] = normalize(vec(axes.asArray()[i]));
        }
        const double r = length(h_);
        box.add(c_ - V3{r, r, r});
        box.add(c_ + V3{r, r, r});
        if (mirror_) {
            box.add({c_.x - r, -c_.y - r, c_.z - r});
            box.add({c_.x + r, -c_.y + r, c_.z + r});
        }
    }
    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 rel = p - c_;
        const double q[3] = {std::fabs(dot(rel, ax_[0])) - (h_.x - round_), std::fabs(dot(rel, ax_[1])) - (h_.y - round_),
                             std::fabs(dot(rel, ax_[2])) - (h_.z - round_)};
        const double out = std::sqrt(std::max(q[0], 0.0) * std::max(q[0], 0.0) + std::max(q[1], 0.0) * std::max(q[1], 0.0) +
                                     std::max(q[2], 0.0) * std::max(q[2], 0.0));
        const double in = std::min(std::max({q[0], q[1], q[2]}), 0.0);
        return {out + in - round_, mat_};
    }

private:
    V3 c_, h_;
    V3 ax_[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double round_;
    bool mirror_;
    int mat_;
};

class Ellipsoid final : public Node {
public:
    explicit Ellipsoid(const Json& j)
        : c_(vec(j, "centre")), r_(vec(j, "radii")), mirror_(j.boolean("mirror", false)), mat_(materialOf(j)) {
        const Json& axes = j.child("axes");
        if (axes.isArray() && axes.asArray().size() == 3) {
            for (std::size_t i = 0; i < 3; ++i) ax_[i] = normalize(vec(axes.asArray()[i]));
        }
        const double m = std::max({r_.x, r_.y, r_.z});
        box.add(c_ - V3{m, m, m});
        box.add(c_ + V3{m, m, m});
        if (mirror_) {
            box.add({c_.x - m, -c_.y - m, c_.z - m});
            box.add({c_.x + m, -c_.y + m, c_.z + m});
        }
    }
    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 rel = p - c_;
        const V3 q{dot(rel, ax_[0]), dot(rel, ax_[1]), dot(rel, ax_[2])};
        const double k0 = length({q.x / r_.x, q.y / r_.y, q.z / r_.z});
        const double k1 = length({q.x / (r_.x * r_.x), q.y / (r_.y * r_.y), q.z / (r_.z * r_.z)});
        if (k1 <= 0.0) return {-std::min({r_.x, r_.y, r_.z}), mat_};
        return {k0 * (k0 - 1.0) / k1, mat_};
    }

private:
    V3 c_, r_;
    V3 ax_[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    bool mirror_;
    int mat_;
};

class Torus final : public Node {
public:
    explicit Torus(const Json& j)
        : c_(vec(j, "centre")), ax_(normalize(vec(j, "axis"))), R_(j.number("R", 0.3)), r_(j.number("r", 0.1)),
          mirror_(j.boolean("mirror", false)), mat_(materialOf(j)) {
        const double m = R_ + r_;
        box.add(c_ - V3{m, m, m});
        box.add(c_ + V3{m, m, m});
        if (mirror_) {
            box.add({c_.x - m, -c_.y - m, c_.z - m});
            box.add({c_.x + m, -c_.y + m, c_.z + m});
        }
    }
    Sample eval(V3 p) const override {
        if (mirror_) p.y = std::fabs(p.y);
        const V3 rel = p - c_;
        const double t = dot(rel, ax_);
        const double rho = length(rel - ax_ * t);
        return {std::hypot(rho - R_, t) - r_, mat_};
    }

private:
    V3 c_, ax_;
    double R_, r_;
    bool mirror_;
    int mat_;
};

// -- operators -------------------------------------------------------------------------------

/// Union, smooth over k: the fillet between its children. A child whose box
/// lies further than the result so far (plus k) cannot change it and is not
/// evaluated - but one whose box holds the point may hold the point, however
/// deep inside the others it is, and always is.
class Union final : public Node {
public:
    Union(std::vector<NodePtr> kids, double k) : kids_(std::move(kids)), k_(k), lb_(kids_.size()) {
        if (kids_.empty()) fail("a union needs children");
        for (const auto& c : kids_) box.merge(c->box);
        box = box.padded(0.5 * k_);
    }

    Sample eval(V3 p) const override {
        const std::size_t n = kids_.size();
        std::vector<double> lb(n);
        std::size_t first = 0;
        for (std::size_t i = 0; i < n; ++i) {
            lb[i] = kids_[i]->box.distance(p);
            if (lb[i] < lb[first]) first = i;
        }
        Sample s = kids_[first]->eval(p);
        double raw = s.d;
        for (std::size_t i = 0; i < n; ++i) {
            if (i == first || (lb[i] > 0.0 && lb[i] >= s.d + k_)) continue;
            const Sample c = kids_[i]->eval(p);
            if (c.d < raw) {
                raw = c.d;
                s.material = c.material;
            }
            s.d = smin(s.d, c.d, k_);
        }
        return s;
    }

private:
    std::vector<NodePtr> kids_;
    double k_;
    std::vector<double> lb_;
};

/// a minus b, smooth over k; the cut's surface takes cut_material (or b's).
class Subtract final : public Node {
public:
    Subtract(NodePtr a, NodePtr b, double k, int cut) : a_(std::move(a)), b_(std::move(b)), k_(k), cut_(cut) {
        box = a_->box;
    }
    Sample eval(V3 p) const override {
        Sample s = a_->eval(p);
        if (b_->box.distance(p) > k_ - s.d) return s; // b is too far to cut here
        const Sample c = b_->eval(p);
        if (-c.d > s.d) s.material = cut_ >= 0 ? cut_ : c.material;
        s.d = smax(s.d, -c.d, k_);
        return s;
    }

private:
    NodePtr a_, b_;
    double k_;
    int cut_;
};

class Intersect final : public Node {
public:
    Intersect(std::vector<NodePtr> kids, double k) : kids_(std::move(kids)), k_(k) {
        if (kids_.empty()) fail("an intersection needs children");
        box = kids_.front()->box;
        for (const auto& c : kids_) box = box.intersect(c->box);
    }
    Sample eval(V3 p) const override {
        Sample s = kids_.front()->eval(p);
        double raw = s.d;
        for (std::size_t i = 1; i < kids_.size(); ++i) {
            const Sample c = kids_[i]->eval(p);
            if (c.d > raw) {
                raw = c.d;
                s.material = c.material;
            }
            s.d = smax(s.d, c.d, k_);
        }
        return s;
    }

private:
    std::vector<NodePtr> kids_;
    double k_;
};

/// Its child, mirrored in the x-z plane (y -> |y|): a pair of anything.
class Mirror final : public Node {
public:
    explicit Mirror(NodePtr kid) : kid_(std::move(kid)) {
        box = kid_->box;
        box.add({box.lo.x, -box.hi.y, box.lo.z});
        box.add({box.hi.x, -box.lo.y, box.hi.z});
    }
    Sample eval(V3 p) const override { return kid_->eval({p.x, std::fabs(p.y), p.z}); }

private:
    NodePtr kid_;
};

/// Its child reflected in the x-z plane (y -> -y): the left one of a pair.
class Reflect final : public Node {
public:
    explicit Reflect(NodePtr kid) : kid_(std::move(kid)) {
        box.add({kid_->box.lo.x, -kid_->box.hi.y, kid_->box.lo.z});
        box.add({kid_->box.hi.x, -kid_->box.lo.y, kid_->box.hi.z});
    }
    Sample eval(V3 p) const override { return kid_->eval({p.x, -p.y, p.z}); }

private:
    NodePtr kid_;
};

/// Its child grown by r (shrunk for r < 0): the gap around a control surface.
class Offset final : public Node {
public:
    Offset(NodePtr kid, double r) : kid_(std::move(kid)), r_(r) { box = kid_->box.padded(std::max(r_, 0.0)); }
    Sample eval(V3 p) const override {
        Sample s = kid_->eval(p);
        s.d -= r_;
        return s;
    }

private:
    NodePtr kid_;
    double r_;
};

/// Its child with its material replaced (a part of a shared solid painted
/// differently).
class Paint final : public Node {
public:
    Paint(NodePtr kid, int material) : kid_(std::move(kid)), mat_(material) { box = kid_->box; }
    Sample eval(V3 p) const override {
        Sample s = kid_->eval(p);
        s.material = mat_;
        return s;
    }

private:
    NodePtr kid_;
    int mat_;
};

NodePtr build(const Json& j, const Scene& scene) {
    if (const Json* op = j.find("op")) {
        const std::string& o = op->asString();
        const double k = j.number("k", 0.0);
        if (o == "union" || o == "intersect") {
            const Json& kids = j.child("children");
            if (!kids.isArray()) fail(o + " needs children");
            std::vector<NodePtr> v;
            for (const auto& c : kids.asArray()) v.push_back(build(c, scene));
            if (o == "union") return std::make_unique<Union>(std::move(v), k);
            return std::make_unique<Intersect>(std::move(v), k);
        }
        if (o == "subtract")
            return std::make_unique<Subtract>(build(j.child("a"), scene), build(j.child("b"), scene), k,
                                              static_cast<int>(j.number("cut_material", -1.0)));
        if (o == "mirror") return std::make_unique<Mirror>(build(j.child("child"), scene));
        if (o == "paint") return std::make_unique<Paint>(build(j.child("child"), scene), materialOf(j));
        if (o == "offset") return std::make_unique<Offset>(build(j.child("child"), scene), j.number("r", 0.0));
        if (o == "reflect") return std::make_unique<Reflect>(build(j.child("child"), scene));
        fail("unknown operator '" + o + "'");
    }
    const Json* prim = j.find("prim");
    if (prim == nullptr) fail("a node needs 'op' or 'prim'");
    const std::string& p = prim->asString();
    if (p == "loft") return std::make_unique<Loft>(j);
    if (p == "wing") return std::make_unique<Wing>(j, scene.foils);
    if (p == "wingregion") return std::make_unique<WingRegion>(j);
    if (p == "revolve") return std::make_unique<Revolve>(j);
    if (p == "capsule") return std::make_unique<Capsule>(j);
    if (p == "cylinder") return std::make_unique<Cylinder>(j);
    if (p == "box") return std::make_unique<BoxPrim>(j);
    if (p == "ellipsoid") return std::make_unique<Ellipsoid>(j);
    if (p == "torus") return std::make_unique<Torus>(j);
    fail("unknown primitive '" + p + "'");
}

} // namespace

FoilTable::FoilTable(const std::vector<double>& x, const std::vector<double>& upper, const std::vector<double>& lower) {
    const std::size_t n = x.size();
    if (n < 3 || upper.size() != n || lower.size() != n) fail("a foil needs matching x, upper and lower lists");
    for (std::size_t i = 1; i < n; ++i)
        if (x[i] <= x[i - 1]) fail("a foil's x must increase");
    // the outline: upper surface from the leading edge back, lower surface forward
    std::vector<double> px, pz;
    for (std::size_t i = 0; i < n; ++i) {
        px.push_back(x[i]);
        pz.push_back(upper[i]);
    }
    for (std::size_t i = n; i-- > 0;) {
        px.push_back(x[i]);
        pz.push_back(lower[i]);
    }
    px.push_back(x[0]);
    pz.push_back(upper[0]);
    nx_ = static_cast<int>(std::lround((kX1 - kX0) / kStep)) + 1;
    nz_ = static_cast<int>(std::lround((kZ1 - kZ0) / kStep)) + 1;
    d_.resize(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(nz_));
    const std::size_t m = px.size();
    for (int k = 0; k < nz_; ++k) {
        const double Z = kZ0 + k * kStep;
        for (int i = 0; i < nx_; ++i) {
            const double X = kX0 + i * kStep;
            double best = 1e30;
            for (std::size_t s = 0; s + 1 < m; ++s) best = std::min(best, segment2(X, Z, px[s], pz[s], px[s + 1], pz[s + 1]));
            const bool inside = X > x.front() && X < x.back() && Z > interp(x, lower, X) && Z < interp(x, upper, X);
            const double d = std::sqrt(best);
            d_[static_cast<std::size_t>(k) * static_cast<std::size_t>(nx_) + static_cast<std::size_t>(i)] =
                static_cast<float>(inside ? -d : d);
        }
    }
}

double FoilTable::sdf(double X, double Z) const {
    const double cx = clamp(X, kX0, kX1), cz = clamp(Z, kZ0, kZ1);
    const double extra = std::hypot(X - cx, Z - cz);
    const double fx = (cx - kX0) / kStep, fz = (cz - kZ0) / kStep;
    const int i = std::min(static_cast<int>(fx), nx_ - 2), k = std::min(static_cast<int>(fz), nz_ - 2);
    const double tx = fx - i, tz = fz - k;
    const std::size_t w = static_cast<std::size_t>(nx_);
    const std::size_t o = static_cast<std::size_t>(k) * w + static_cast<std::size_t>(i);
    const double a = static_cast<double>(d_[o]), b = static_cast<double>(d_[o + 1]);
    const double c = static_cast<double>(d_[o + w]), e = static_cast<double>(d_[o + w + 1]);
    return lerp(lerp(a, b, tx), lerp(c, e, tx), tz) + extra;
}

Scene buildScene(const Json& description) {
    Scene scene;
    const Json& foils = description.child("foils");
    if (foils.isArray()) {
        for (const auto& f : foils.asArray())
            scene.foils.push_back(std::make_unique<FoilTable>(list(f, "x"), list(f, "upper"), list(f, "lower")));
    }
    scene.root = build(description.child("root"), scene);
    return scene;
}

} // namespace meshkit
