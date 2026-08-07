using System.Runtime.InteropServices;
using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

static double[] Values(object? value) => value is double[] array
    ? array
    : ((Array)(value ?? Array.Empty<double>())).Cast<object>().Select(Convert.ToDouble).ToArray();

static double[] Point(double[] t, double[] p)
{
    var s = t[12];
    return new[] {
        s * (t[0] * p[0] + t[3] * p[1] + t[6] * p[2]) + t[9],
        s * (t[1] * p[0] + t[4] * p[1] + t[7] * p[2]) + t[10],
        s * (t[2] * p[0] + t[5] * p[1] + t[8] * p[2]) + t[11]
    };
}

static double[] Direction(double[] t, double[] p)
{
    var result = new[] {
        t[0] * p[0] + t[3] * p[1] + t[6] * p[2],
        t[1] * p[0] + t[4] * p[1] + t[7] * p[2],
        t[2] * p[0] + t[5] * p[1] + t[8] * p[2]
    };
    var norm = Math.Sqrt(result.Sum(v => v * v));
    return result.Select(v => v / norm).ToArray();
}

var project = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", ".."));
var assemblyPath = Path.Combine(project, "零件", "完整零件", "组合无人机.SLDASM");
var outputPath = Path.Combine(project, "drone_sim_ws", "analysis", "cad_direct", "axis_face_probe.json");
SldWorks? app = null;
ModelDoc2? document = null;
try
{
    app = new SldWorks { Visible = false, CommandInProgress = true };
    var errors = 0;
    var warnings = 0;
    document = app.OpenDoc6(
        assemblyPath,
        (int)swDocumentTypes_e.swDocASSEMBLY,
        (int)swOpenDocOptions_e.swOpenDocOptions_Silent |
        (int)swOpenDocOptions_e.swOpenDocOptions_ReadOnly,
        "", ref errors, ref warnings);
    if (document is null) throw new InvalidOperationException($"OpenDoc6 failed: {errors}/{warnings}");
    var assembly = (AssemblyDoc)document;
    assembly.ResolveAllLightWeightComponents(false);
    var records = new List<object>();
    foreach (Component2 component in (object[])(assembly.GetComponents(false) ?? Array.Empty<object>()))
    {
        var instance = component.Name2;
        var path = component.GetPathName() ?? "";
        var filename = Path.GetFileName(path);
        var isFlightRotor =
            filename.Equals("f电机.SLDPRT", StringComparison.OrdinalIgnoreCase) ||
            filename.Equals("f螺旋桨.SLDPRT", StringComparison.OrdinalIgnoreCase);
        var isArmDriveHorn =
            instance.Contains("fSO101 Assembly-1", StringComparison.Ordinal) &&
            instance.Contains("金属舵盘（驱动）", StringComparison.Ordinal);
        if (!isFlightRotor && !isArmDriveHorn) continue;
        if (component.Transform2 is not MathTransform transform) continue;
        var matrix = Values(transform.ArrayData);
        var cylinders = new List<object>();
        var bodies = component.GetBodies2((int)swBodyType_e.swSolidBody) as object[] ?? Array.Empty<object>();
        for (var bodyIndex = 0; bodyIndex < bodies.Length; ++bodyIndex)
        {
            var body = (Body2)bodies[bodyIndex];
            var faces = body.GetFaces() as object[] ?? Array.Empty<object>();
            for (var faceIndex = 0; faceIndex < faces.Length; ++faceIndex)
            {
                var face = (Face2)faces[faceIndex];
                var surface = face.GetSurface() as Surface;
                if (surface is null || !surface.IsCylinder()) continue;
                var values = Values(surface.CylinderParams);
                var origin = values[..3];
                var axis = values[3..6];
                cylinders.Add(new {
                    body_index = bodyIndex,
                    face_index = faceIndex,
                    local_origin_m = origin,
                    local_axis = axis,
                    radius_m = values[6],
                    area_m2 = face.GetArea(),
                    assembly_origin_m = Point(matrix, origin),
                    assembly_axis = Direction(matrix, axis)
                });
            }
        }
        var box = Values(component.GetBox(false, false));
        records.Add(new {
            instance,
            path,
            assembly_bbox_m = box,
            assembly_bbox_center_m = new[] {
                (box[0] + box[3]) / 2.0,
                (box[1] + box[4]) / 2.0,
                (box[2] + box[5]) / 2.0
            },
            cylinders
        });
        Console.WriteLine($"{component.Name2}: {cylinders.Count} cylindrical faces");
    }
    Directory.CreateDirectory(Path.GetDirectoryName(outputPath)!);
    File.WriteAllText(outputPath, JsonSerializer.Serialize(records, new JsonSerializerOptions { WriteIndented = true }));
    Console.WriteLine(outputPath);
}
finally
{
    if (document is not null && app is not null) app.CloseDoc(document.GetTitle());
    if (app is not null)
    {
        app.CommandInProgress = false;
        app.ExitApp();
        Marshal.FinalReleaseComObject(app);
    }
}
