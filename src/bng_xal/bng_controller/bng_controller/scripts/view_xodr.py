import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import math
from typing import List, Tuple, Dict, Optional


class OpenDriveParser:
    """Enhanced parser for OpenDRIVE (.xodr) files with support for parametric cubic curves."""

    def __init__(self, xodr_file_path: str):
        self.xodr_file_path = xodr_file_path
        self.roads = []
        self.junctions = []

    def parse(self):
        """Parse the OpenDRIVE file and extract road data."""
        try:
            tree = ET.parse(self.xodr_file_path)
            root = tree.getroot()

            # Parse roads
            for road_elem in root.findall("road"):
                road_data = self._parse_road(road_elem)
                if road_data:
                    self.roads.append(road_data)

            # Parse junctions (optional)
            for junction_elem in root.findall("junction"):
                junction_data = self._parse_junction(junction_elem)
                if junction_data:
                    self.junctions.append(junction_data)

        except ET.ParseError as e:
            print(f"Error parsing XML: {e}")
        except FileNotFoundError:
            print(f"File not found: {self.xodr_file_path}")

    def _parse_road(self, road_elem) -> Optional[Dict]:
        """Parse a single road element."""
        road_id = road_elem.get("id")
        road_name = road_elem.get("name", f"Road_{road_id}")
        length = float(road_elem.get("length", 0))

        # Find planView element
        plan_view = road_elem.find("planView")
        if plan_view is None:
            return None

        geometries = []
        for geom_elem in plan_view.findall("geometry"):
            geometry = self._parse_geometry(geom_elem)
            if geometry:
                geometries.append(geometry)

        return {
            "id": road_id,
            "name": road_name,
            "length": length,
            "geometries": geometries,
        }

    def _parse_geometry(self, geom_elem) -> Optional[Dict]:
        """Parse geometry elements including parametric cubic curves."""
        s = float(geom_elem.get("s", 0))
        x = float(geom_elem.get("x", 0))
        y = float(geom_elem.get("y", 0))
        hdg = float(geom_elem.get("hdg", 0))  # heading
        length = float(geom_elem.get("length", 0))

        geometry = {"s": s, "x": x, "y": y, "hdg": hdg, "length": length}

        # Check for different geometry types
        line_elem = geom_elem.find("line")
        arc_elem = geom_elem.find("arc")
        spiral_elem = geom_elem.find("spiral")
        param_poly3_elem = geom_elem.find("paramPoly3")
        poly3_elem = geom_elem.find("poly3")

        if line_elem is not None:
            geometry["type"] = "line"
        elif arc_elem is not None:
            geometry["type"] = "arc"
            geometry["curvature"] = float(arc_elem.get("curvature", 0))
        elif spiral_elem is not None:
            geometry["type"] = "spiral"
            geometry["curvStart"] = float(spiral_elem.get("curvStart", 0))
            geometry["curvEnd"] = float(spiral_elem.get("curvEnd", 0))
        elif param_poly3_elem is not None:
            geometry["type"] = "paramPoly3"
            # Parse parametric cubic curve parameters
            geometry["aU"] = float(param_poly3_elem.get("aU", 0))
            geometry["bU"] = float(param_poly3_elem.get("bU", 0))
            geometry["cU"] = float(param_poly3_elem.get("cU", 0))
            geometry["dU"] = float(param_poly3_elem.get("dU", 0))
            geometry["aV"] = float(param_poly3_elem.get("aV", 0))
            geometry["bV"] = float(param_poly3_elem.get("bV", 0))
            geometry["cV"] = float(param_poly3_elem.get("cV", 0))
            geometry["dV"] = float(param_poly3_elem.get("dV", 0))
            geometry["pRange"] = param_poly3_elem.get("pRange", "normalized")
        elif poly3_elem is not None:
            geometry["type"] = "poly3"
            # Parse cubic polynomial parameters
            geometry["a"] = float(poly3_elem.get("a", 0))
            geometry["b"] = float(poly3_elem.get("b", 0))
            geometry["c"] = float(poly3_elem.get("c", 0))
            geometry["d"] = float(poly3_elem.get("d", 0))
        else:
            geometry["type"] = "unknown"
            print(f"Unknown geometry type found in element at s={s}")

        return geometry

    def _parse_junction(self, junction_elem) -> Optional[Dict]:
        """Parse junction elements."""
        junction_id = junction_elem.get("id")
        junction_name = junction_elem.get("name", f"Junction_{junction_id}")

        return {"id": junction_id, "name": junction_name}

    def generate_road_coordinates(
        self, road: Dict, resolution: float = 1.0
    ) -> Tuple[List[float], List[float]]:
        """Generate coordinate points along the road reference line."""
        x_coords = []
        y_coords = []

        all_x = []
        all_y = []

        # First pass: collect all points
        for geometry in road["geometries"]:
            geom_x, geom_y = self._generate_geometry_coordinates(geometry, resolution)
            all_x.extend(geom_x)
            all_y.extend(geom_y)

        if not all_x or not all_y:
            return [], []

        x_coords = all_x
        y_coords = all_y

        return x_coords, y_coords

    def _generate_geometry_coordinates(
        self, geometry: Dict, resolution: float
    ) -> Tuple[List[float], List[float]]:
        """Generate coordinates for a specific geometry segment."""
        start_x = geometry["x"]
        start_y = geometry["y"]
        hdg = geometry["hdg"]
        length = geometry["length"]
        geom_type = geometry["type"]

        # For very short segments, just use endpoints
        if length < 0.01:
            return [start_x], [start_y]

        # Number of points based on resolution
        num_points = max(int(length / resolution), 2)

        x_coords = []
        y_coords = []

        if geom_type == "line":
            # Straight line
            s_values = np.linspace(0, length, num_points)
            for s in s_values:
                x = start_x + s * math.cos(hdg)
                y = start_y + s * math.sin(hdg)
                x_coords.append(x)
                y_coords.append(y)

        elif geom_type == "arc":
            # Circular arc
            curvature = geometry["curvature"]
            if abs(curvature) < 1e-10:
                return self._generate_line_coordinates(
                    start_x, start_y, hdg, length, num_points
                )

            radius = 1.0 / curvature
            center_x = start_x - radius * math.sin(hdg)
            center_y = start_y + radius * math.cos(hdg)

            s_values = np.linspace(0, length, num_points)
            for s in s_values:
                angle = hdg + s * curvature
                x = center_x + radius * math.sin(angle)
                y = center_y - radius * math.cos(angle)
                x_coords.append(x)
                y_coords.append(y)

        elif geom_type == "spiral":
            # Clothoid/Euler spiral
            curv_start = geometry["curvStart"]
            curv_end = geometry["curvEnd"]

            s_values = np.linspace(0, length, num_points)
            for s in s_values:
                # Linear interpolation of curvature
                curvature = curv_start + (curv_end - curv_start) * (s / length)

                # Simplified spiral calculation
                if s == 0:
                    x = start_x
                    y = start_y
                else:
                    current_hdg = hdg + curvature * s * 0.5
                    x = start_x + s * math.cos(current_hdg)
                    y = start_y + s * math.sin(current_hdg)

                x_coords.append(x)
                y_coords.append(y)

        elif geom_type == "paramPoly3":
            # Parametric cubic curve (BeamNG.tech likely uses this)
            aU, bU, cU, dU = (
                geometry["aU"],
                geometry["bU"],
                geometry["cU"],
                geometry["dU"],
            )
            aV, bV, cV, dV = (
                geometry["aV"],
                geometry["bV"],
                geometry["cV"],
                geometry["dV"],
            )
            p_range = geometry["pRange"]

            # Determine parameter range
            if p_range == "arcLength":
                p_values = np.linspace(0, length, num_points)
            else:  # normalized
                p_values = np.linspace(0, 1, num_points)

            cos_hdg = math.cos(hdg)
            sin_hdg = math.sin(hdg)

            for p in p_values:
                # Calculate local u,v coordinates
                u = aU + bU * p + cU * p**2 + dU * p**3
                v = aV + bV * p + cV * p**2 + dV * p**3

                # Transform to world coordinates
                x = start_x + u * cos_hdg - v * sin_hdg
                y = start_y + u * sin_hdg + v * cos_hdg

                x_coords.append(x)
                y_coords.append(y)

        elif geom_type == "poly3":
            # Cubic polynomial (deprecated but still supported)
            a, b, c, d = geometry["a"], geometry["b"], geometry["c"], geometry["d"]

            u_values = np.linspace(0, length, num_points)
            cos_hdg = math.cos(hdg)
            sin_hdg = math.sin(hdg)

            for u in u_values:
                # Calculate v coordinate using polynomial
                v = a + b * u + c * u**2 + d * u**3

                # Transform to world coordinates
                x = start_x + u * cos_hdg - v * sin_hdg
                y = start_y + u * sin_hdg + v * cos_hdg

                x_coords.append(x)
                y_coords.append(y)

        else:
            # Unknown geometry type - treat as line
            return self._generate_line_coordinates(
                start_x, start_y, hdg, length, num_points
            )

        return x_coords, y_coords

    def _generate_line_coordinates(
        self, start_x: float, start_y: float, hdg: float, length: float, num_points: int
    ) -> Tuple[List[float], List[float]]:
        """Generate coordinates for a straight line."""
        x_coords = []
        y_coords = []

        s_values = np.linspace(0, length, num_points)
        for s in s_values:
            x = start_x + s * math.cos(hdg)
            y = start_y + s * math.sin(hdg)
            x_coords.append(x)
            y_coords.append(y)

        return x_coords, y_coords

    def plot_roads(
        self,
        figsize: Tuple[int, int] = (12, 8),
        resolution: float = 0.5,
        show_road_names: bool = True,
        show_direction: bool = True,
        show_geometry_info: bool = False,
    ):
        """Plot all roads using matplotlib with improved handling of parametric curves."""
        plt.figure(figsize=figsize)

        colors = plt.cm.tab10(np.linspace(0, 1, len(self.roads)))

        for i, road in enumerate(self.roads):
            x_coords, y_coords = self.generate_road_coordinates(road, resolution)

            if x_coords and y_coords:
                # Plot road centerline
                label = f"{road['name']} (ID: {road['id']})"
                if show_geometry_info:
                    poly3_count = sum(
                        1 for g in road["geometries"] if g["type"] == "paramPoly3"
                    )
                    if poly3_count > 0:
                        label += f" [{poly3_count} curves]"

                plt.plot(x_coords, y_coords, color=colors[i], linewidth=2, label=label)

                # Show road direction with arrow
                if show_direction and len(x_coords) > 1:
                    mid_idx = len(x_coords) // 2
                    if mid_idx + 1 < len(x_coords):
                        dx = x_coords[mid_idx + 1] - x_coords[mid_idx]
                        dy = y_coords[mid_idx + 1] - y_coords[mid_idx]
                        length_arrow = math.sqrt(dx * dx + dy * dy)
                        if length_arrow > 0:
                            scale = 3.0 / length_arrow
                            plt.arrow(
                                x_coords[mid_idx],
                                y_coords[mid_idx],
                                dx * scale,
                                dy * scale,
                                head_width=1,
                                head_length=1.5,
                                fc=colors[i],
                                ec=colors[i],
                            )

                # Show road name at start
                if show_road_names:
                    plt.annotate(
                        f"{road['name']}",
                        (x_coords[0], y_coords[0]),
                        xytext=(5, 5),
                        textcoords="offset points",
                        fontsize=9,
                        color=colors[i],
                        bbox=dict(
                            boxstyle="round,pad=0.3", facecolor="white", alpha=0.7
                        ),
                    )

        plt.xlabel("X Coordinate (m)")
        plt.ylabel("Y Coordinate (m)")
        title = f"OpenDRIVE Road Network\n{self.xodr_file_path}"
        plt.title(title)
        plt.grid(True, alpha=0.3)
        plt.axis("equal")
        plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
        plt.tight_layout()
        plt.show()

    def get_road_info(self) -> str:
        """Get detailed information about the parsed roads."""
        info = f"OpenDRIVE File: {self.xodr_file_path}\n"
        info += f"Number of roads: {len(self.roads)}\n"
        info += f"Number of junctions: {len(self.junctions)}\n\n"

        for road in self.roads:
            info += f"Road {road['id']} ({road['name']}):\n"
            info += f"  Length: {road['length']:.2f} m\n"
            info += f"  Geometries: {len(road['geometries'])}\n"

            # Count geometry types
            type_counts = {}
            for geom in road["geometries"]:
                geom_type = geom["type"]
                type_counts[geom_type] = type_counts.get(geom_type, 0) + 1

            info += f"  Geometry types: {type_counts}\n"

            # Show first few geometries as examples
            for j, geom in enumerate(road["geometries"][:5]):
                info += f"    {j+1}. {geom['type'].capitalize()} - Length: {geom['length']:.3f} m"
                if geom["type"] == "paramPoly3":
                    info += f" (bU={geom['bU']:.3f}, cU={geom['cU']:.6f})"
                info += "\n"

            if len(road["geometries"]) > 5:
                info += f"    ... and {len(road['geometries']) - 5} more\n"
            info += "\n"

        return info


def main():
    """Example usage with enhanced BeamNG.tech support."""
    # Your BeamNG.tech exported file
    xodr_file_path = ""

    if not xodr_file_path:
        xodr_file_path = input("Enter path to OpenDRIVE file: ")
        if not xodr_file_path:
            print("No file path provided. Exiting.")
            return

    # Create parser and parse file
    parser = OpenDriveParser(xodr_file_path)
    parser.parse()

    # Print detailed road information
    print(parser.get_road_info())
    parser.plot_roads(figsize=(15, 10), resolution=0.2, show_geometry_info=True)


if __name__ == "__main__":
    main()
