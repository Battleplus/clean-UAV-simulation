import fs from "node:fs/promises";

const { SpreadsheetFile, Workbook } = await import(
  "file:///C:/Users/ASUS/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/@oai/artifact-tool/dist/artifact_tool.mjs"
);

const outputDir = "E:/清洁无人机/outputs/019fb1d4-2ca6-7e03-83ad-50c3d54b6d4e";
const outputPath = `${outputDir}/当前最需要参数_单页.xlsx`;
const previewPath = `${outputDir}/previews/当前最需要参数_单页.png`;

await fs.mkdir(`${outputDir}/previews`, { recursive: true });

const wb = Workbook.create();
const ws = wb.worksheets.add("当前必填");
ws.showGridLines = false;
ws.freezePanes.freezeRows(4);

const navy = "#17365D";
const blue = "#2F75B5";
const paleBlue = "#DDEBF7";
const yellow = "#FFF2CC";
const red = "#FCE4D6";
const green = "#E2F0D9";
const grid = "#C9D6E4";

function title(range, text) {
  range.merge();
  range.values = [[text]];
  range.format = {
    fill: navy,
    font: { bold: true, color: "#FFFFFF", size: 17 },
    horizontalAlignment: "center",
    verticalAlignment: "center",
  };
  range.format.rowHeight = 34;
}

function section(range, text) {
  range.merge();
  range.values = [[text]];
  range.format = {
    fill: navy,
    font: { bold: true, color: "#FFFFFF" },
    verticalAlignment: "center",
  };
  range.format.rowHeight = 24;
}

function header(range) {
  range.format = {
    fill: blue,
    font: { bold: true, color: "#FFFFFF" },
    horizontalAlignment: "center",
    verticalAlignment: "center",
    wrapText: true,
    borders: { preset: "all", style: "thin", color: grid },
  };
  range.format.rowHeight = 32;
}

function dataBlock(range) {
  range.format = {
    borders: { preset: "all", style: "thin", color: grid },
    verticalAlignment: "center",
  };
}

title(ws.getRange("A1:L1"), "my_drone 当前最需要参数（只填黄色格）");
ws.getRange("A3:L3").merge();
ws.getRange("A3").values = [[
  "用途：解除机械臂固定、建立八旋翼推力分配，并让 PX4 仿真具备可校准的前后左右飞行。坐标统一使用机体 FRD：X 前、Y 右、Z 下。",
]];
ws.getRange("A3:L3").format = {
  fill: paleBlue,
  font: { italic: true, color: navy },
  verticalAlignment: "center",
};
ws.getRange("A3:L3").format.rowHeight = 28;

ws.getRange("A4:F4").values = [[
  "分组", "必填项目", "填写值", "单位", "填写说明", "状态",
]];
header(ws.getRange("A4:F4"));

const basic = [
  ["整机", "最终起飞总质量", null, "kg", "含电池、机械臂和末端工具", null],
  ["整机", "重心 X", null, "m", "相对机体原点，向前为正", null],
  ["整机", "重心 Y", null, "m", "相对机体原点，向右为正", null],
  ["整机", "重心 Z", null, "m", "相对机体原点，向下为正", null],
  ["整机", "整机 CAD/STEP/URDF 路径", null, "-", "用于获得惯量；若没有，提供 Ixx/Iyy/Izz 也可", null],
  ["推力", "单电机最大推力", null, "N", "同一电池电压与桨叶条件", null],
  ["推力", "推力系数 kf", null, "N/(rad/s)²", "若没有可提供油门-RPM-推力表", null],
  ["推力", "反扭矩系数 km", null, "N·m/(rad/s)²", "若没有可暂填未知", null],
];
ws.getRange("A5:F12").values = basic;
ws.getRange("F5:F12").formulas = Array.from({ length: 8 }, (_, i) => [
  `=IF(C${i + 5}="","待填写","已填写")`,
]);
dataBlock(ws.getRange("A5:F12"));
ws.getRange("C5:C12").format.fill = yellow;
ws.getRange("A5:B12").format.fill = paleBlue;
ws.getRange("E5:E12").format.wrapText = true;

section(ws.getRange("A14:L14"), "八个电机安装与倾角（8 行全部填写）");
ws.getRange("A15:L15").values = [[
  "电机", "X m", "Y m", "Z m", "推力轴 X", "推力轴 Y", "推力轴 Z",
  "倾角 °", "旋向", "最大推力 N", "kf", "状态",
]];
header(ws.getRange("A15:L15"));
const motors = Array.from({ length: 8 }, (_, i) => [i + 1, null, null, null, null, null, null, null, null, null, null, null]);
ws.getRange("A16:L23").values = motors;
ws.getRange("L16:L23").formulas = Array.from({ length: 8 }, (_, i) => [
  `=IF(COUNTA(B${i + 16}:K${i + 16})=10,"已填写","待填写")`,
]);
dataBlock(ws.getRange("A16:L23"));
ws.getRange("B16:K23").format.fill = yellow;
ws.getRange("A16:A23").format.fill = paleBlue;
ws.getRange("I16:I23").dataValidation = { rule: { type: "list", values: ["CW", "CCW"] } };
ws.getRange("B16:K23").format.numberFormat = "0.000";

section(ws.getRange("A25:L25"), "机械臂关节（解除固定最关键）");
ws.getRange("A26:L26").values = [[
  "关节序号", "URDF/SDF 关节名", "轴 X", "轴 Y", "轴 Z", "下限 rad", "上限 rad",
  "连续力矩 N·m", "最大速度 rad/s", "飞行中运动", "初始/悬停角 rad", "状态",
]];
header(ws.getRange("A26:L26"));
const jointNames = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];
ws.getRange("A27:L32").values = jointNames.map((name, i) => [i + 1, name, null, null, null, null, null, null, null, null, null, null]);
ws.getRange("L27:L32").formulas = jointNames.map((_, i) => [
  `=IF(COUNTA(B${i + 27}:K${i + 27})=10,"已填写","待填写")`,
]);
dataBlock(ws.getRange("A27:L32"));
ws.getRange("C27:K32").format.fill = yellow;
ws.getRange("A27:B32").format.fill = paleBlue;
ws.getRange("J27:J32").dataValidation = { rule: { type: "list", values: ["是", "否"] } };
ws.getRange("C27:I32").format.numberFormat = "0.000";

section(ws.getRange("A34:H34"), "机械臂连杆最低质量参数");
ws.getRange("A35:H35").values = [[
  "序号", "连杆名称", "质量 kg", "COM X m", "COM Y m", "COM Z m", "CAD/STEP 路径", "状态",
]];
header(ws.getRange("A35:H35"));
const links = [
  "arm_base_link", "shoulder_link", "upper_arm_link", "lower_arm_link",
  "wrist_link", "gripper_link", "cleaning_head/payload",
];
ws.getRange("A36:H42").values = links.map((name, i) => [i + 1, name, null, null, null, null, null, null]);
ws.getRange("H36:H42").formulas = links.map((_, i) => [
  `=IF(OR(G${i + 36}<>"",COUNTA(C${i + 36}:F${i + 36})=4),"已填写","待填写")`,
]);
dataBlock(ws.getRange("A36:H42"));
ws.getRange("C36:G42").format.fill = yellow;
ws.getRange("A36:B42").format.fill = paleBlue;
ws.getRange("C36:F42").format.numberFormat = "0.000";

ws.getRange("J35:L35").values = [["完成情况", "已完成", "总数"]];
header(ws.getRange("J35:L35"));
ws.getRange("J36:J39").values = [["基础参数"], ["八旋翼"], ["机械臂关节"], ["机械臂连杆"]];
ws.getRange("K36:K39").formulas = [
  ['=COUNTIF(F5:F12,"已填写")'],
  ['=COUNTIF(L16:L23,"已填写")'],
  ['=COUNTIF(L27:L32,"已填写")'],
  ['=COUNTIF(H36:H42,"已填写")'],
];
ws.getRange("L36:L39").values = [[8], [8], [6], [7]];
ws.getRange("J40").values = [["总计"]];
ws.getRange("K40").formulas = [["=SUM(K36:K39)"]];
ws.getRange("L40").formulas = [["=SUM(L36:L39)"]];
ws.getRange("J36:L40").format = {
  fill: paleBlue,
  borders: { preset: "all", style: "thin", color: grid },
};
ws.getRange("J40:L40").format.font = { bold: true, color: navy };

for (const range of ["F5:F12", "L16:L23", "L27:L32", "H36:H42"]) {
  ws.getRange(range).conditionalFormats.add("containsText", {
    text: "已填写",
    format: { fill: green, font: { color: "#006100" } },
  });
  ws.getRange(range).conditionalFormats.add("containsText", {
    text: "待填写",
    format: { fill: red, font: { color: "#C00000" } },
  });
}

const widths = {
  A: 11, B: 23, C: 14, D: 15, E: 29, F: 13,
  G: 16, H: 15, I: 13, J: 15, K: 15, L: 13,
};
for (const [col, width] of Object.entries(widths)) {
  ws.getRange(`${col}:${col}`).format.columnWidth = width;
}
ws.getRange("A1:L42").format.font = { name: "Microsoft YaHei", size: 10 };
ws.getRange("A1:L42").format.verticalAlignment = "center";

console.log("KEY_RANGE");
console.log((await wb.inspect({
  kind: "table",
  range: "当前必填!A1:L42",
  include: "values,formulas",
  tableMaxRows: 42,
  tableMaxCols: 12,
  maxChars: 7000,
})).ndjson);

console.log("FORMULA_ERRORS");
console.log((await wb.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 100 },
  summary: "final formula error scan",
})).ndjson);

const preview = await wb.render({
  sheetName: "当前必填",
  range: "A1:L42",
  scale: 1.2,
  format: "png",
});
await fs.writeFile(previewPath, new Uint8Array(await preview.arrayBuffer()));

const xlsx = await SpreadsheetFile.exportXlsx(wb);
await xlsx.save(outputPath);
console.log(`OUTPUT=${outputPath}`);
