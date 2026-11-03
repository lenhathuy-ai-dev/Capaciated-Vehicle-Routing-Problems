"""Convert CVRP benchmark instances (.vrp/.txt/.dat, common VRP-REP XML) to compact TXT.

Output format:
  N K CAPACITY OPTIMAL
  x_depot y_depot 0
  x_customer1 y_customer1 demand1
  ...
N counts all locations INCLUDING the depot. -1 = unknown.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

DATA_DIR = Path(r"D:\_2026\1.CNTT\7.metaheuristic\01_VRP_Vehicle_Routing_Problem\01_CVRP_Capacitated_Vehicle_Routing_Problem\data")
OUT_DIR = DATA_DIR.parent / "new_data"
EXTENSIONS = {".vrp", ".txt", ".dat", ".xml"}


def numeric(value):
    """Return a compact string, e.g. 82 rather than 82.0."""
    return format(float(value), ".12g")


def find_k(*texts):
    for s in texts:
        if not s:
            continue
        m = re.search(r"(?:^|[-_])k(\d+)(?=$|[-_.\s])", s, re.I)
        if not m:
            m = re.search(r"(?:no\.?\s*of\s*trucks?|number\s*of\s*(?:trucks?|vehicles?)|vehicles?|trucks?)\s*[:=]\s*(\d+)", s, re.I)
        if m:
            return int(m.group(1))
    return -1


def find_optimal(*texts):
    """Only parse a value explicitly labelled optimal, never assume BKS = optimum."""
    for s in texts:
        if not s:
            continue
        m = re.search(r"\b(?:optimal\s+(?:value|cost|solution|distance|objective)?|optimum)\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)", s, re.I)
        if m:
            return numeric(m.group(1))
    return "-1"


def parse_tsplib(path):
    headers = {}
    coords, demands, depots = {}, {}, []
    section = None
    text = path.read_text(encoding="utf-8-sig", errors="replace")

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        upper = line.upper()
        if upper == "EOF":
            break
        if upper in {"NODE_COORD_SECTION", "DEMAND_SECTION", "DEPOT_SECTION", "EDGE_WEIGHT_SECTION", "DISPLAY_DATA_SECTION", "TOUR_SECTION"}:
            section = upper
            continue
        if section is None:
            m = re.match(r"^([A-Za-z_]+)\s*:?\s+(.+)$", line)
            if m:
                headers[m.group(1).upper()] = m.group(2).strip()
        elif section == "NODE_COORD_SECTION":
            parts = line.split()
            if len(parts) >= 3:
                coords[int(parts[0])] = (numeric(parts[1]), numeric(parts[2]))
        elif section == "DEMAND_SECTION":
            parts = line.split()
            if len(parts) >= 2:
                demands[int(parts[0])] = numeric(parts[1])
        elif section == "DEPOT_SECTION":
            for part in line.split():
                depot_id = int(part)
                if depot_id == -1:
                    break
                depots.append(depot_id)

    if headers.get("TYPE", "CVRP").upper() != "CVRP":
        raise ValueError("Không phải CVRP")
    if not coords or not demands or "CAPACITY" not in headers:
        raise ValueError("Thiếu tọa độ, nhu cầu hoặc CAPACITY (có thể file không đúng định dạng TSPLIB)")
    if len(depots) != 1:
        raise ValueError("Không xác định được đúng một kho (DEPOT_SECTION)")
    depot = depots[0]
    if depot not in coords or depot not in demands:
        raise ValueError("Thiếu thông tin kho")
    if set(coords) != set(demands):
        raise ValueError("Các điểm trong NODE_COORD_SECTION và DEMAND_SECTION không khớp")
    if "DIMENSION" in headers and len(coords) != int(headers["DIMENSION"]):
        raise ValueError("Số tọa độ không khớp DIMENSION")
    name = headers.get("NAME", path.stem)
    k = find_k(name, path.stem, headers.get("COMMENT", ""))
    opt = find_optimal(headers.get("COMMENT", ""), headers.get("OPTIMAL", ""), headers.get("OPTIMUM", ""))
    ordered = [depot] + sorted(i for i in coords if i != depot)
    points = [(coords[i][0], coords[i][1], "0" if i == depot else demands[i]) for i in ordered]
    return len(points), k, numeric(headers["CAPACITY"]), opt, points


def xml_tag(tag):
    return tag.split("}")[-1].lower()


def xml_children(el, tag):
    return [e for e in el.iter() if xml_tag(e.tag) == tag]


def xml_value(el, name, default=None):
    for child in el:
        if xml_tag(child.tag) == name:
            return (child.text or "").strip()
    return el.attrib.get(name, default)


def parse_xml(path):
    """Common VRP-REP XML: network/nodes/node, requests/request, fleet/vehicle_profile."""
    root = ET.parse(path).getroot()
    coords, demands = {}, {}
    for node in xml_children(root, "node"):
        nid = node.get("id")
        x, y = xml_value(node, "cx"), xml_value(node, "cy")
        if nid is not None and x is not None and y is not None:
            coords[int(nid)] = (numeric(x), numeric(y))
            d = xml_value(node, "demand")
            if d is not None:
                demands[int(nid)] = float(d)

    if not coords:
        raise ValueError("XML không có network/nodes/node/cx/cy kiểu VRP-REP")

    for request in xml_children(root, "request"):
        nid = request.get("node")
        q = xml_value(request, "quantity")
        if nid is not None and q is not None:
            demands[int(nid)] = demands.get(int(nid), 0) + float(q)

    profiles = xml_children(root, "vehicle_profile")
    if not profiles:
        raise ValueError("XML thiếu vehicle_profile/capacity")
    capacities = {numeric(xml_value(p, "capacity")) for p in profiles if xml_value(p, "capacity") is not None}
    if len(capacities) != 1:
        raise ValueError("Không phải CVRP tải trọng đồng nhất hoặc thiếu capacity")
    capacity = capacities.pop()

    depot_vals = {int(xml_value(p, "departure_node")) for p in profiles if xml_value(p, "departure_node") is not None}
    if len(depot_vals) != 1:
        raise ValueError("Không xác định được đúng một kho từ departure_node")
    depot = depot_vals.pop()
    if depot not in coords:
        raise ValueError("Kho không có tọa độ")
    if set(demands) - set(coords) or set(coords) - set(demands) - {depot}:
        raise ValueError("Thiếu nhu cầu của một số khách hàng")

    # A VRP-REP instance may leave vehicle quantity unspecified (unlimited fleet).
    counts = [xml_value(p, "number") or xml_value(p, "count") for p in profiles]
    k = sum(int(c) for c in counts) if all(c is not None and c.isdigit() for c in counts) else -1
    name = next((e.text for e in xml_children(root, "name") if e.text), path.stem)
    if k == -1:
        k = find_k(name, path.stem)
    comments = " ".join((e.text or "") for e in xml_children(root, "comment"))
    opt = find_optimal(comments)
    ordered = [depot] + sorted(i for i in coords if i != depot)
    points = [(coords[i][0], coords[i][1], "0" if i == depot else numeric(demands[i])) for i in ordered]
    return len(points), k, capacity, opt, points


def main():
    if not DATA_DIR.is_dir():
        print(f"Không tìm thấy thư mục: {DATA_DIR}")
        return

    converted, skipped = 0, []
    for src in sorted(DATA_DIR.rglob("*")):
        if not src.is_file() or src.suffix.lower() not in EXTENSIONS:
            continue
        try:
            n, k, capacity, optimal, points = parse_xml(src) if src.suffix.lower() == ".xml" else parse_tsplib(src)
            output = OUT_DIR / src.relative_to(DATA_DIR).with_suffix(".txt")
            output.parent.mkdir(parents=True, exist_ok=True)
            lines = [f"{n} {k} {capacity} {optimal}"]
            lines.extend(f"{x} {y} {q}" for x, y, q in points)
            output.write_text("\n".join(lines) + "\n", encoding="utf-8")
            converted += 1
        except (ValueError, KeyError, OSError, ET.ParseError, OverflowError) as exc:
            skipped.append(f"{src.relative_to(DATA_DIR)}: {exc}")

    print(f"Hoàn tất: {converted} file -> {OUT_DIR}")
    print("Ghi chú: -1 nghĩa là không tìm thấy K hoặc giá trị tối ưu trong file nguồn.")
    if skipped:
        log = OUT_DIR / "conversion_skipped.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text("\n".join(skipped) + "\n", encoding="utf-8")
        print(f"Đã bỏ qua {len(skipped)} file; xem {log}")


if __name__ == "__main__":
    main()
