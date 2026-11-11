"""Render a KiCad .kicad_sch to SVG/PNG with matplotlib, using KiCad's default colours.
Usage: python render_schematic.py kicad/smart_cane.kicad_sch schematic
"""
import re, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

C_WIRE = "#009600"; C_BODY = "#840000"; C_FILL = "#FFFFC2"; C_PIN = "#840000"
C_PNAME = "#006464"; C_PNUM = "#A90000"; C_REF = "#006464"; C_VAL = "#006464"
C_LABEL = "#000000"; C_SHEET = "#840000"

def parse(s):
    tok = re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', s); st = [[]]
    for x in tok:
        if x == '(':
            st.append([])
        elif x == ')':
            v = st.pop(); st[-1].append(v)
        else:
            st[-1].append(x[1:-1].replace('\\"', '"') if x.startswith('"') else x)
    return st[0][0]

def kids(n, name): return [c for c in n if isinstance(c, list) and c and c[0] == name]
def kid(n, name):
    k = kids(n, name); return k[0] if k else None
def is_hidden(n):
    if 'hide' in n: return True
    h = kid(n, 'hide')
    if h is not None and (len(h) == 1 or h[1] == 'yes'): return True
    e = kid(n, 'effects')
    if e is not None:
        if 'hide' in e: return True
        h = kid(e, 'hide')
        if h is not None and (len(h) == 1 or h[1] == 'yes'): return True
    return False
def font_size(n, default=1.27):
    e = kid(n, 'effects')
    if e is None: return default
    f = kid(e, 'font'); s = kid(f, 'size') if f else None
    return float(s[1]) if s else default
def justify(n):
    e = kid(n, 'effects'); j = kid(e, 'justify') if e else None
    return j[1:] if j else []

PT = 72 / 25.4 * 1.32   # mm text height -> points (KiCad size ~ cap height)

def main(src, out):
    d = parse(open(src, encoding='utf8').read())
    libs = {kid_[1]: kid_ for kid_ in kid(d, 'lib_symbols')[1:]}

    fig = plt.figure(figsize=(297 / 25.4, 210 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 297); ax.set_ylim(210, 0); ax.axis('off')
    fig.patch.set_facecolor('white')

    def text(x, y, s, size, color, ang=0, ha='center', va='center', bold=False):
        ax.text(x, y, s, fontsize=size * PT, color=color, rotation=ang, ha=ha, va=va,
                family='DejaVu Sans', rotation_mode='anchor', fontweight='bold' if bold else 'normal')

    # ---- drawing sheet (A4 frame, zone ticks, title block)
    for m in (10, 12):
        ax.add_patch(Rectangle((m, m), 297 - 2 * m, 210 - 2 * m, fill=False, ec=C_SHEET, lw=0.5))
    for i in range(6):
        x0 = 12 + i * (273 / 6)
        for yy in (11, 199):
            text(x0 + 273 / 12, yy, str(i + 1), 1.3, C_SHEET)
        if i: ax.plot([x0, x0], [10, 12], color=C_SHEET, lw=0.5); ax.plot([x0, x0], [198, 200], color=C_SHEET, lw=0.5)
    for i, ch in enumerate("ABCD"):
        y0 = 12 + i * (186 / 4)
        for xx in (11, 286):
            text(xx, y0 + 186 / 8, ch, 1.3, C_SHEET)
        if i: ax.plot([10, 12], [y0, y0], color=C_SHEET, lw=0.5); ax.plot([285, 287], [y0, y0], color=C_SHEET, lw=0.5)
    tb_x, tb_y = 177, 166
    ax.add_patch(Rectangle((tb_x, tb_y), 285 - tb_x, 198 - tb_y, fill=False, ec=C_SHEET, lw=0.5))
    for yy in (178, 186, 192):
        ax.plot([tb_x, 285], [yy, yy], color=C_SHEET, lw=0.5)
    text(tb_x + 2, 172, "SafeRoute Smart Cane - Pod Schematic", 2.0, C_SHEET, ha='left', bold=True)
    text(tb_x + 2, 182, "File: smart_cane.kicad_sch", 1.3, C_SHEET, ha='left')
    text(tb_x + 2, 189, "Size: A4", 1.3, C_SHEET, ha='left')
    text(tb_x + 40, 189, "Sheet: 1/1", 1.3, C_SHEET, ha='left')
    text(tb_x + 2, 195, "KiCad E.D.A. 10.0", 1.3, C_SHEET, ha='left')

    # ---- wires
    for w in kids(d, 'wire'):
        pts = [(float(p[1]), float(p[2])) for p in kid(w, 'pts')[1:]]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=C_WIRE, lw=0.6, solid_capstyle='round')

    # ---- labels (local)
    for l in kids(d, 'label'):
        at = kid(l, 'at'); x, y, a = float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0
        s = font_size(l); off = 0.3
        if a == 0: text(x, y - off, l[1], s, C_LABEL, ha='left', va='bottom')
        elif a == 180: text(x, y - off, l[1], s, C_LABEL, ha='right', va='bottom')
        elif a == 90: text(x - off, y, l[1], s, C_LABEL, ang=90, ha='left', va='bottom')
        else: text(x - off, y, l[1], s, C_LABEL, ang=90, ha='right', va='bottom')

    # ---- symbols
    for inst in kids(d, 'symbol'):
        ln = kid(inst, 'lib_name'); lid = ln[1] if ln else kid(inst, 'lib_id')[1]; lib = libs[lid]
        at = kid(inst, 'at'); X, Y, R = float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0
        mir = kid(inst, 'mirror'); mir = mir[1] if mir else None
        unit = int(kid(inst, 'unit')[1]) if kid(inst, 'unit') else 1
        ra = math.radians(R)

        def T(px, py):
            py = -py
            if mir == 'x': py = -py
            if mir == 'y': px = -px
            return (X + px * math.cos(ra) + py * math.sin(ra), Y - px * math.sin(ra) + py * math.cos(ra))
        def Tdir(dx, dy):
            a = T(dx, dy); o = T(0, 0); return (a[0] - o[0], a[1] - o[1])

        pn = kid(lib, 'pin_names'); pnum = kid(lib, 'pin_numbers')
        name_off = float(kid(pn, 'offset')[1]) if pn is not None and kid(pn, 'offset') else 0.508
        names_hidden = pn is not None and is_hidden(pn)
        nums_hidden = pnum is not None and is_hidden(pnum)
        is_power = kid(lib, 'power') is not None

        units = [u for u in kids(lib, 'symbol')]
        for u in units:
            m = re.search(r'_(\d+)_(\d+)$', u[1]); un, st = int(m.group(1)), int(m.group(2))
            if un not in (0, unit) or st not in (0, 1): continue
            for g in u[1:]:
                if not isinstance(g, list): continue
                kind = g[0]
                stroke_w = 0.25
                fill = kid(g, 'fill'); ftype = kid(fill, 'type')[1] if fill and kid(fill, 'type') else 'none'
                fc = C_FILL if ftype == 'background' else (C_BODY if ftype == 'outline' else 'none')
                if kind == 'rectangle':
                    s = kid(g, 'start'); e = kid(g, 'end')
                    a = T(float(s[1]), float(s[2])); b = T(float(e[1]), float(e[2]))
                    ax.add_patch(Rectangle((min(a[0], b[0]), min(a[1], b[1])), abs(a[0] - b[0]), abs(a[1] - b[1]),
                                           ec=C_BODY, fc=fc, lw=0.7, zorder=1))
                elif kind == 'text_box':
                    s = kid(g, 'at'); z = kid(g, 'size')
                    x0, y0 = float(s[1]), float(s[2]); x1, y1 = x0 + float(z[1]), y0 + float(z[2])
                    a = T(x0, y0); b = T(x1, y1)
                    ax.add_patch(Rectangle((min(a[0], b[0]), min(a[1], b[1])), abs(a[0] - b[0]), abs(a[1] - b[1]),
                                           ec=C_BODY, fc=fc, lw=0.7, zorder=1))
                    if g[1]:
                        text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, g[1], font_size(g), C_BODY)
                elif kind == 'polyline':
                    pts = [T(float(p[1]), float(p[2])) for p in kid(g, 'pts')[1:]]
                    if fc != 'none' and len(pts) > 2:
                        ax.add_patch(Polygon(pts, closed=True, ec=C_BODY, fc=fc, lw=0.7, zorder=2))
                    else:
                        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=C_BODY, lw=0.7, zorder=2)
                elif kind == 'circle':
                    c = kid(g, 'center'); r = float(kid(g, 'radius')[1]); p = T(float(c[1]), float(c[2]))
                    ax.add_patch(Circle(p, r, ec=C_BODY, fc=fc, lw=0.7, zorder=2))
                elif kind == 'text':
                    if is_hidden(g): continue
                    p = kid(g, 'at'); q = T(float(p[1]), float(p[2]))
                    text(q[0], q[1], g[1], font_size(g), C_BODY)
                elif kind == 'pin':
                    if is_hidden(g): continue
                    p = kid(g, 'at'); px, py, pa = float(p[1]), float(p[2]), float(p[3])
                    L = float(kid(g, 'length')[1])
                    dx, dy = math.cos(math.radians(pa)), math.sin(math.radians(pa))
                    a = T(px, py); b = T(px + L * dx, py + L * dy)
                    ax.plot([a[0], b[0]], [a[1], b[1]], color=C_PIN, lw=0.7, zorder=3)
                    sd = Tdir(dx, dy)  # screen direction into body
                    horiz = abs(sd[0]) > abs(sd[1])
                    name = kid(g, 'name')[1]; num = kid(g, 'number')[1]
                    nsz = font_size(kid(g, 'name')); msz = font_size(kid(g, 'number'))
                    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                    if not names_hidden and name not in ('', '~'):
                        if name_off > 0:
                            q = (b[0] + sd[0] * name_off, b[1] + sd[1] * name_off)
                            if horiz:
                                text(q[0], q[1], name, nsz, C_PNAME, ha='left' if sd[0] > 0 else 'right')
                            else:
                                text(q[0], q[1], name, nsz, C_PNAME, ang=90, ha='left' if sd[1] < 0 else 'right')
                        else:
                            if horiz: text(mid[0], mid[1] - 0.4, name, nsz, C_PNAME, va='bottom')
                            else: text(mid[0] - 0.4, mid[1], name, nsz, C_PNAME, ang=90, va='bottom')
                    if not nums_hidden and not is_power:
                        if horiz: text(mid[0], mid[1] - 0.4, num, msz, C_PNUM, va='bottom')
                        else: text(mid[0] - 0.4, mid[1], num, msz, C_PNUM, ang=90, va='bottom')

        # fields
        for prop in kids(inst, 'property'):
            if prop[1] not in ('Reference', 'Value') or is_hidden(prop) or not prop[2]: continue
            if prop[1] == 'Reference' and prop[2].startswith('#'): continue
            p = kid(prop, 'at'); x, y, a = float(p[1]), float(p[2]), float(p[3]) if len(p) > 3 else 0
            j = justify(prop)
            ha = 'left' if 'left' in j else ('right' if 'right' in j else 'center')
            va = 'top' if 'top' in j else ('bottom' if 'bottom' in j else 'center')
            ang = 90 if round(a) % 180 == 90 else 0
            text(x, y, prop[2], font_size(prop), C_REF if prop[1] == 'Reference' else C_VAL, ang=ang, ha=ha, va=va)

    fig.savefig(out + '.svg', facecolor='white')
    fig.savefig(out + '.png', dpi=300, facecolor='white')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
