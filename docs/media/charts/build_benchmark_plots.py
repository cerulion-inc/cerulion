#!/usr/bin/env python3
"""Render the two benchmark figures of the root README from the published result packages.

It replots retained measurements. It never runs a benchmark and never edits a package.

Run it from the repository root:

    uv run --quiet --python 3.12 --with matplotlib==3.10.6 --with fonttools python docs/media/charts/build_benchmark_plots.py

It writes docs/media/native-rtt.svg and docs/media/platform-rtt.svg, and puts the PNG twins and
a manifest of every source, point, floor and output hash under docs/media/charts/out/ (ignored
by git). It refuses to run when anything under docs/benchmarks/results differs from HEAD, so a
figure always describes a committed package.

Inputs, all in the tree:
  docs/benchmarks/results/            the packages: CSVs, run.json, raw .bin samples, .rate sidecars
  expected-chart-points.json          the reviewed point ledger: every plotted p50 and p99, per series
  reviewed-point-hashes.json          one sha256 per figure over its plotted points
  rmw-same-harness-point-ledger.json  the rmw_cerulion series, recomputed independently from the raw samples

What it asserts before it draws: each CSV's sha256 and every plotted point equal the ledger; the
rmw_cerulion CSV has the columns of the stock ROS 2 CSV, and its run.json records the machine,
variant, governor, turbo and idle-state posture of the native campaign's stock ROS 2 invocation;
the 50 retained raw .bin files re-reduce to the CSV (session medians, pooled p99, pooled minimum)
and the 50 .rate sidecars agree; the drawn artists carry the verified values; each figure's point
hash equals reviewed-point-hashes.json.

Typography: layout uses the macOS system faces (Helvetica Neue and Menlo), so the script runs on
macOS. The SVG text is then re-pointed at GitHub's font stacks.

Figure style: GitHub README typography, the in-plot mark key, filled p50 dots, p50 to p99 bands,
floors recorded and not drawn, all ten payload sizes labelled, "lower is better" after the title
in the brand blue, the p50 at the last payload printed at the right end of every line in the
series colour, no rate strip.
"""
from __future__ import annotations
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import struct
import subprocess
import tempfile
import textwrap

import matplotlib
from matplotlib.backends.backend_agg import FigureCanvasAgg
matplotlib.use('Agg')
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.offsetbox import AnchoredOffsetbox, DrawingArea, HPacker, TextArea, VPacker
from matplotlib.patches import Circle, Rectangle
from matplotlib.ticker import FixedLocator, FuncFormatter, NullLocator

HERE = Path(__file__).resolve().parent
# This file lives at docs/media/charts/, so the repository root is three levels up. Every path
# below is inside the repository.
SOURCE = HERE.parents[2]
LEDGER_PATH = HERE/'expected-chart-points.json'
PACKET_MANIFEST = HERE/'reviewed-point-hashes.json'
ASSETS = HERE.parent          # docs/media: the two SVGs the README embeds
OUT = HERE/'out'              # PNG twins and the manifest; ignored by git
RESULTS = SOURCE/'docs/benchmarks/results'
HEROES = RESULTS/'8a84baf25d5d1710-2026-09-16-fixed100-heroes'
RMW_SAME = RESULTS/'8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion'
RMW_SAME_RUN = RMW_SAME/'jazzy-cerulion-loan-k5'
RMW_SAME_CELL = 'jazzy_cerulion_shm_loan_be1_chrt0'
RMW_SAME_LEDGER = HERE/'rmw-same-harness-point-ledger.json'
HEROES_STOCK_ROS2_CELL = 'jazzy_stock_rclcpp_chrt0'
JETSON = RESULTS/'9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx'
MAC = RESULTS/'0b7bc3994f78e232-2026-09-17-fixed100-apple-m4'
EXPECTED_SIZES = [64,256,1024,4096,16384,65536,262144,1048576,4194304,16777216]
SIZE_LABELS = ['64 B','256 B','1 KiB','4 KiB','16 KiB','64 KiB','256 KiB','1 MiB','4 MiB','16 MiB']
COLORS = dict(multi='#2563EB', single='#089D9E', stock='#D36B26', composed='#74708C', zenoh='#B58900',
              rmw='#A21CAF', jetson='#D36B26', mac='#D1242F')
TEXT = '#172B45'
MUTED = '#53677C'
GRID = '#E3EAF1'
FRAME = '#CCD7E3'
STRIP_BG = '#F6F8FA'
STRIP_BAR = '#C5D0DC'
KEY_INK = '#4B5F78'
DASH = dict(solid='-', dashed=(0,(6,3)), shortdash=(0,(3,2.4)), dashdot=(0,(6,2.5,1.8,2.5)), dotted=(0,(1.4,2.6)))

# --- typography -----------------------------------------------------------------
LAYOUT_SANS = ['Helvetica Neue', 'Helvetica']
LAYOUT_MONO = ['Menlo']
GITHUB_SANS = "-apple-system, BlinkMacSystemFont, 'Segoe UI', 'Noto Sans', Helvetica, Arial, sans-serif"
GITHUB_MONO = "ui-monospace, SFMono-Regular, 'SF Mono', Menlo, Consolas, 'Liberation Mono', monospace"
CODE_TOKENS = ['rmw_fastrtps_cpp', 'rmw_cerulion', 'UInt8MultiArray', 'rclcpp']
SIZE = dict(title=20, subtitle=13, legend=13.5, axis_title=13.5, tick=12.5, footnote=11.5,
            strip=11.5, strip_header=12, key=13, endlabel=12.5, brand=10)

def register_layout_faces():
    """matplotlib reads only face 0 of a .ttc, so bold text would lay out and rasterize
    with the regular face. Split the system collections into per-face files in a temp
    dir and register them; the family names are unchanged so the SVG text is identical."""
    try:
        from fontTools.ttLib import TTCollection
    except ImportError:
        return {'registered': [], 'note': 'fonttools unavailable; collection face 0 only'}
    cache = Path(tempfile.mkdtemp(prefix='cerulion-layout-faces-'))
    registered = []
    for ttc in ['/System/Library/Fonts/HelveticaNeue.ttc', '/System/Library/Fonts/Menlo.ttc']:
        path = Path(ttc)
        if not path.exists():
            continue
        for font in TTCollection(str(path)).fonts:
            family = font['name'].getDebugName(1)
            style = font['name'].getDebugName(2)
            if style not in ('Regular', 'Bold'):
                continue
            out = cache/f'{family}-{style}.ttf'.replace(' ', '_')
            font.save(str(out))
            fm.fontManager.addfont(str(out))
            registered.append(f'{family} {style}')
    return {'registered': registered, 'note': 'faces registered from the system collections for layout only'}

LAYOUT_FACES = register_layout_faces()
assert 'Helvetica Neue Bold' in LAYOUT_FACES['registered'], LAYOUT_FACES

plt.rcParams.update({
    'font.family':LAYOUT_SANS, 'font.size':SIZE['tick'], 'text.color':TEXT,
    'axes.labelcolor':TEXT, 'xtick.color':MUTED, 'ytick.color':MUTED,
    'axes.edgecolor':FRAME, 'axes.spines.top':False, 'axes.spines.right':False,
    'axes.titleweight':'bold', 'axes.labelsize':SIZE['axis_title'], 'xtick.labelsize':SIZE['tick'],
    'ytick.labelsize':SIZE['tick'], 'figure.facecolor':'white', 'axes.facecolor':'white',
    'savefig.facecolor':'white', 'svg.fonttype':'none',
    'svg.hashsalt':'cerulion-reviewed-benchmarks-7fc1a639-v5',
    'lines.solid_capstyle':'round', 'legend.frameon':False,
})
manifest = {
    'scope':'Replots of retained measurements from the published result packages; no new measurement and no edit to a package.',
    'source_tree':None,
    'renderer_sha256':None,
    'python_version':platform.python_version(), 'matplotlib_version':matplotlib.__version__,
    'multi_process_series':{
        'display_label':'Cerulion multi-process (default)',
        'configuration':'Two declared process groups, CERULION_EXECUTION_MODE=free_run.',
        'see':'docs/PERFORMANCE.md states which execution mode each column ran and carries the lockstep rows.',
    },
    'floor_definition':{
        'native_and_platform':'floor_ns column of the verified CSV row: compile_csv.py stats["floor"] = s[0], the minimum of the pooled per-sample round trips of the cell (all sessions).',
        'rmw_cerulion_on_the_main_chart':'floor_ns column of the same-harness package CSV (the same compile_csv.py definition as the native rows), asserted equal to the pooled minimum of the 50 retained raw .bin files.',
    },
    'typography':{
        'layout_faces':LAYOUT_FACES,
        'svg_sans_stack':GITHUB_SANS, 'svg_mono_stack':GITHUB_MONO,
        'mono_tokens':CODE_TOKENS, 'sizes_px':SIZE,
    },
    'sources':[], 'figures':{},
}

OUT.mkdir(exist_ok=True)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def git(*args):
    return subprocess.run(['git','-C',str(SOURCE),*args],check=True,capture_output=True,text=True).stdout.strip()
# Pin what was read: the commit, and that nothing under the results directory differs from it.
assert git('status','--porcelain','--','docs/benchmarks/results')=='', 'uncommitted change under docs/benchmarks/results'
manifest['source_tree']={'head':git('rev-parse','HEAD'),'results_dir_clean_at_head':True}
def csv_rows(p):
    return list(csv.DictReader(x for x in p.read_text().splitlines() if not x.startswith('#')))
def rel(path):
    return str(path.relative_to(SOURCE)) if path.is_relative_to(SOURCE) else str(path.relative_to(HERE))
def record_source(path, **extra):
    row={'path':rel(path),'sha256':sha(path),**extra}
    if row not in manifest['sources']: manifest['sources'].append(row)

ledger = json.loads(LEDGER_PATH.read_text())
manifest['independent_point_ledger_sha256']=sha(LEDGER_PATH)
packet_manifest = json.loads(PACKET_MANIFEST.read_text())
manifest['reviewed_point_hashes_sha256']=sha(PACKET_MANIFEST)

def verified_series(chart):
    output=[]
    for item in ledger['charts'][chart]['series']:
        current=SOURCE/item['source_csv']
        # The ledger pins the committed CSV byte for byte, and every plotted point below.
        assert sha(current)==item['source_sha256'], f'CSV changed since the ledger was reviewed: {item["source_csv"]}'
        rows=csv_rows(current)
        assert [int(r['payload_bytes']) for r in rows]==EXPECTED_SIZES
        actual=[]; floors=[]
        for row, point in zip(rows,item['points'],strict=True):
            p={
                'payload_bytes':int(row['payload_bytes']),
                'p50_ns':int(row[item['metric_columns'][0]]),
                'p99_ns':int(row[item['metric_columns'][1]]),
                'samples':int(row['iterations']),
                'repetitions':int(row['rep_count']),
                'achieved_rate_hz':row['achieved_rate_hz'],
            }
            assert p==point, f'Independent point mismatch: {item["label"]}, {p}'
            assert 0<p['p50_ns']<=p['p99_ns']
            floor=int(row['floor_ns'])
            assert 0<floor<=p['p50_ns'], (item['label'], p, floor)
            actual.append(p); floors.append({'payload_bytes':p['payload_bytes'],'floor_ns':floor})
        record_source(current,ledger_source_sha256=item['source_sha256'])
        output.append({'historical_label':item['label'],'source':str(current.relative_to(SOURCE)),'points':actual,
                       'v5':{'floor':floors,'floor_source':{'path':rel(current),'column':'floor_ns','sha256':sha(current)}}})
    return output

native=verified_series('native-rtt.svg')
platforms=verified_series('platform-rtt.svg')

# The same-harness rmw_cerulion row: ROS 2 Jazzy over rmw_cerulion, loaned publishing and
# loaned takes, measured by benches/latency like the stock and composed ROS 2 rows above.
def type7_truncated(sorted_values, q):
    """The suite's percentile (compile_csv.py percentile): linear interpolation, truncated to
    a whole nanosecond. Written out here so the raw re-reduction does not import the suite."""
    n=len(sorted_values)
    if n==1: return sorted_values[0]
    rank=q*(n-1); lo=int(rank); hi=min(lo+1,n-1)
    if lo==hi: return sorted_values[lo]
    return int(sorted_values[lo]+(rank-lo)*(sorted_values[hi]-sorted_values[lo]))

def run_posture(run_json, cell):
    """(variant, governor, turbo, idle-state posture) of every invocation that ran `cell`, plus the
    machine hash; one value each or the package is not one posture."""
    run=json.loads(run_json.read_text())
    invocations=[i for i in run['invocations'] if any(c['cell']==cell for c in i['cells'])]
    assert invocations, (run_json,cell)
    assert all(c['outcome']=='ok' for i in invocations for c in i['cells'] if c['cell']==cell), run_json
    posture={(i['variant'],i['cpu_governor'],i['turbo_boost'],json.dumps(i['dma_lock_posture'])) for i in invocations}
    machine={i['machine_hash'] for i in invocations}|{run['machine_hash']}
    assert len(posture)==1 and len(machine)==1, (run_json,posture,machine)
    return {'variant':next(iter(posture))[0],'cpu_governor':next(iter(posture))[1],'turbo_boost':next(iter(posture))[2],
            'dma_lock_posture':json.loads(next(iter(posture))[3]),'machine_hash':machine.pop(),'invocations':len(invocations)}

def same_harness_rmw_series():
    csv_path=RMW_SAME_RUN/f'results_{RMW_SAME_CELL}.csv'
    point_ledger=json.loads(RMW_SAME_LEDGER.read_text())
    # 1. the ledger pins the CSV, as expected-chart-points.json pins the native CSVs
    assert rel(csv_path)==point_ledger['source_csv'] and sha(csv_path)==point_ledger['source_sha256'], 'same-harness rmw CSV changed'
    assert point_ledger['metric_columns']==['round_trip_p50_ns','round_trip_p99_ns']
    # 2. one schema: the CSV has exactly the columns of the native campaign's stock ROS 2 CSV
    stock_csv=HEROES/'hero-stock'/f'results_{HEROES_STOCK_ROS2_CELL}.csv'
    assert list(csv_rows(csv_path)[0].keys())==list(csv_rows(stock_csv)[0].keys()), 'CSV schema differs from the stock ROS 2 row'
    # 3. one machine, one posture, one pacing: run.json against the native campaign's stock ROS 2 invocations
    posture=run_posture(RMW_SAME_RUN/'run.json',RMW_SAME_CELL)
    stock_posture=run_posture(HEROES/'hero-stock'/'run.json',HEROES_STOCK_ROS2_CELL)
    same=['variant','cpu_governor','turbo_boost','dma_lock_posture','machine_hash','invocations']
    assert {k:posture[k] for k in same}=={k:stock_posture[k] for k in same}, (posture,stock_posture)
    assert posture['variant']=='fixed100' and posture['machine_hash']==HEROES.name.split('-')[0]==RMW_SAME.name.split('-')[0]
    rows=csv_rows(csv_path)
    assert [int(r['payload_bytes']) for r in rows]==EXPECTED_SIZES
    points=[]; floors=[]; raw_files=[]
    for row,expected in zip(rows,point_ledger['points'],strict=True):
        size=int(row['payload_bytes'])
        p={'payload_bytes':size,'p50_ns':int(row['round_trip_p50_ns']),'p99_ns':int(row['round_trip_p99_ns']),
           'samples':int(row['iterations']),'repetitions':int(row['rep_count']),'achieved_rate_hz':row['achieved_rate_hz']}
        floor=int(row['floor_ns'])
        # 4. every plotted point equals the independently recomputed ledger point
        assert p=={k:expected[k] for k in p}, f'point ledger mismatch: rmw_cerulion, {p}'
        assert floor==expected['floor_ns'] and 0<floor<=p['p50_ns']<=p['p99_ns'], (size,floor,p)
        # 5. the row is the fully loaned cell at the full commanded rate, with nothing unaccounted
        assert (row['loaned'],row['chrt'],row['achieved_rate_hz'],row['unstamped'],row['nonpositive_rtt'])==('1','0','100','0','0'), row
        assert (p['samples'],p['repetitions'])==(10000,5), p
        # 6. the generator re-reduces the retained raw samples: five sessions of 2,000 u64 nanosecond values
        sessions=[]
        for rep in range(1,p['repetitions']+1):
            raw=RMW_SAME_RUN/f'rep{rep}'/'raw'/f'{RMW_SAME_CELL}_{size}.bin'
            data=raw.read_bytes(); assert len(data)==8*2000, (raw,len(data))
            sessions.append(sorted(struct.unpack(f'<{len(data)//8}Q',data)))
            rate=(raw.with_suffix('.rate')).read_text().splitlines()[0].strip()
            assert rate==row['achieved_rate_hz'], (raw,rate)
            raw_files.append({'path':rel(raw),'sha256':sha(raw),'samples':len(data)//8,'rate_sidecar_hz':rate})
        pooled=sorted(v for s in sessions for v in s)
        session_p50s=[type7_truncated(s,.5) for s in sessions]
        assert session_p50s==expected['rep_p50s_ns'], (size,session_p50s)
        assert type7_truncated(sorted(session_p50s),.5)==p['p50_ns'], (size,session_p50s,p)
        assert type7_truncated(pooled,.99)==p['p99_ns'] and pooled[0]==floor and len(pooled)==p['samples'], (size,p)
        assert (min(session_p50s),max(session_p50s))==(int(row['rep_p50_min_ns']),int(row['rep_p50_max_ns']))
        points.append(p); floors.append({'payload_bytes':size,'floor_ns':floor})
    record_source(csv_path,role='the rmw_cerulion series on the main chart: same-harness campaign, fully loaned cell')
    record_source(RMW_SAME_LEDGER,role='point ledger for the rmw_cerulion series',points_from=point_ledger['points_from'])
    record_source(RMW_SAME_RUN/'run.json',role='posture and machine of the rmw_cerulion series')
    record_source(HEROES/'hero-stock'/'run.json',role='posture and machine of the stock ROS 2 row the rmw_cerulion posture is asserted equal to')
    per_session={p['samples']//p['repetitions'] for p in points}; assert per_session=={2000}
    meta={'loaded_like':'verified_series: same compile_csv.py schema and metric columns as the stock and composed ROS 2 rows; p50 = median of the five session p50s, p99 pooled',
          'points_equal_point_ledger':True,'csv_columns_equal_stock_ros2_csv':True,
          'posture':posture,'posture_equals_native_campaign_stock_ros2_invocations':True,
          'sessions':5,'samples_per_session':per_session.pop(),'loaned':'1 on every row','achieved_rate_hz':'100 on every row and in all 50 .rate sidecars',
          'raw_rereduction':{'files':len(raw_files),'samples':sum(f['samples'] for f in raw_files),
                             'equals_csv':'session p50s, median of session p50s, pooled p99, pooled minimum, per-session p50 spread','raw_files':raw_files}}
    assert meta['raw_rereduction']['files']==50 and meta['raw_rereduction']['samples']==100000
    return ({'historical_label':RMW_SAME_CELL,'source':rel(csv_path),'points':points,
             'v5':{'floor':floors,'floor_source':{'path':rel(csv_path),'column':'floor_ns','sha256':sha(csv_path)}}},meta)

rmw_same,rmw_same_meta=same_harness_rmw_series()

# The raw iceoryx2 floor rows the platform footnote cites (numbers read, never typed).
def floor_row_p50_range(path):
    rows=csv_rows(path)
    assert [int(r['payload_bytes']) for r in rows]==EXPECTED_SIZES
    assert all(r['achieved_rate_hz']=='100' and r['rep_count']=='5' for r in rows), path
    record_source(path,role='raw iceoryx2 floor row cited in the platform footnote')
    return [int(r['round_trip_p50_ns']) for r in rows]
x86_floor=floor_row_p50_range(HEROES/'iox2-rerun/results_iox2_chrt0.csv')
jetson_floor=floor_row_p50_range(JETSON/'results_iox2_chrt0.csv')
mac_floor_rows=csv_rows(MAC/'results_iox2_chrt0.csv')
assert all(r['iterations']=='0' and r['round_trip_p50_ns']=='' for r in mac_floor_rows), 'Mac floor row unexpectedly populated'
record_source(MAC/'results_iox2_chrt0.csv',role='Mac raw iceoryx2 floor row: empty (fixed 100 Hz not sustained), so no Mac floor is cited')

def us(ns): return ns/1000
def fmt_range_us(values_ns):
    return f'{us(min(values_ns)):.1f} to {us(max(values_ns)):.1f} µs'
def date_of(package_dir):
    m=re.search(r'(\d{4}-\d{2}-\d{2})',package_dir.name); return m.group(1)

# --- layout in points --------------------------------------------------------------
FIG_W_IN = 13
FIG_W_PT = FIG_W_IN*72
LEFT, RIGHT = 0.1, .905          # the right margin carries the line-end values
TITLE_TOP, SUBTITLE_TOP, SUB_PITCH = 30, 58, 17
LEGEND_ROW_PT, KEY_ROW_PT = 21, 26
FOOT_PITCH, FOOT_LAST_TOP, BRAND_Y = 16.5, 34, 12
ENDLABEL_MIN_SEP_PT = 15

def header_height(sub_lines, legend_rows):
    legend_top = SUBTITLE_TOP + SUB_PITCH*(sub_lines-1) + 14 + 12
    key_top = legend_top + legend_rows*LEGEND_ROW_PT + 8
    return legend_top, key_top, key_top + KEY_ROW_PT + 24

def bottom_height(foot_lines):
    # x tick labels (16) + label pad (10) + axis title (16) + gap (10) + footer + brand
    return 52 + FOOT_LAST_TOP + FOOT_PITCH*(foot_lines-1) + 14

def strip_height(exception_rows, text_lines=1):
    return 6 + 14 + 6 + 13 + 4 + 4 + exception_rows*(4+13*text_lines+4) + 6

CURVES_DRAWN=[]   # per-figure draw order, asserted against the series list at save time
def make_figure(top_pt, main_pt, strip_pt, bottom_pt):
    CURVES_DRAWN.clear()
    assert strip_pt==0, 'the rate strip is retired'
    height_pt = top_pt + main_pt + bottom_pt
    fig = plt.figure(figsize=(FIG_W_IN, height_pt/72))
    ax = fig.add_axes([LEFT, bottom_pt/height_pt, RIGHT-LEFT, main_pt/height_pt])
    ax.set_gid('data-axes-0')
    return fig, ax, None, height_pt

def payload_axis(ax, strip):
    ax.set_xscale('log',base=2)
    ax.set_xlim(43,26000000)
    ax.xaxis.set_major_locator(FixedLocator(EXPECTED_SIZES))
    ax.set_xticklabels(SIZE_LABELS)
    ax.xaxis.set_minor_locator(NullLocator())
    ax.spines['bottom'].set_color(FRAME)
    ax.tick_params(axis='both',which='both',length=0,pad=8)
    ax.grid(axis='y',which='major',color=GRID,linewidth=.9)
    ax.grid(axis='x',which='major',color=GRID,linewidth=.7,alpha=.55)
    ax.set_axisbelow(True)
    ax.set_xlabel('Payload size (log scale)',labelpad=10)

def latency_axis(ax, ymin, ymax, ticks):
    ax.set_yscale('log'); ax.set_ylim(ymin,ymax)
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v,_:f'{v/1000:g} ms' if v>=1000 else f'{v:g} µs'))
    ax.set_ylabel('Round-trip latency',labelpad=12)

def fmt_value(ns):
    v=ns/1000
    return f'{v/1000:.3g} ms' if v>=1000 else f'{v:.3g} µs'

def curve(ax, series, label, color, linestyle='-', zorder=3):
    """Band p50..p99, line through the p50s, filled p50 dot. The per-point floor is
    verified against the p50 and recorded, not drawn."""
    x=[p['payload_bytes'] for p in series['points']]
    y=[us(p['p50_ns']) for p in series['points']]
    hi=[us(p['p99_ns']) for p in series['points']]
    floors=[us(f['floor_ns']) for f in series['v5']['floor']]
    assert [f['payload_bytes'] for f in series['v5']['floor']]==x
    index=len(CURVES_DRAWN); CURVES_DRAWN.append(label)
    ax.fill_between(x,y,hi,color=color,alpha=.11,linewidth=0,zorder=zorder-1)
    # Floors are read and recorded in the manifest but not drawn: the bars cluttered the plots.
    assert all(f<=yi for yi,f in zip(y,floors,strict=True))
    line,=ax.plot(x,y,color=color,lw=2.2,marker='o',markersize=6.4,
        markerfacecolor=color,markeredgecolor='white',markeredgewidth=.9,label=label,zorder=zorder+1,
        linestyle=linestyle)
    return line

def end_labels(ax, entries):
    """Print each series' last p50 (16 MiB, or the last measured payload for a series that
    stops earlier) at the right end of its line in the series colour. Labels closer than
    ENDLABEL_MIN_SEP_PT are pushed apart; a moved label gets a thin leader so the eye still
    finds its line."""
    height_pt=ax.get_position().height*ax.figure.get_figheight()*72
    lo,hi=ax.get_ylim()
    def to_frac(v): return (math.log10(v)-math.log10(lo))/(math.log10(hi)-math.log10(lo))
    def to_val(f): return 10**(f*(math.log10(hi)-math.log10(lo))+math.log10(lo))
    sep=ENDLABEL_MIN_SEP_PT/height_pt
    order=sorted(range(len(entries)),key=lambda i:entries[i][0])
    entries=[e if len(e)==4 else (*e,16777216) for e in entries]
    pos=[to_frac(entries[i][0]) for i in order]
    for _ in range(200):
        moved=False
        for a in range(len(pos)-1):
            gap=pos[a+1]-pos[a]
            if gap<sep-1e-9:
                push=(sep-gap)/2; pos[a]-=push; pos[a+1]+=push; moved=True
        if not moved: break
    placed=[]
    for k,i in enumerate(order):
        y,color,text,x_last=entries[i]
        yl=to_val(pos[k]); x_label=x_last*2**.36
        if abs(pos[k]-to_frac(y))>1e-6:
            ax.plot([x_last*2**.12,x_label*2**-.06],[y,yl],color=color,lw=.8,alpha=.6,clip_on=False,zorder=2,gid=f'end-leader:{k}')
        ax.text(x_label,yl,text,color=color,fontsize=SIZE['endlabel'],ha='left',va='center',clip_on=False,
                fontweight='bold',gid=f'end-label:{k}')
        placed.append({'label':text,'value_us':float(y),'placed_at_us':float(yl),'at_payload_bytes':x_last,'moved':bool(abs(pos[k]-to_frac(y))>1e-6)})
    return placed

def rate_of(point):
    """The achieved_rate_hz column of the verified CSV row. A string, verbatim."""
    return point['achieved_rate_hz']
def rate_text(value):
    return f'{int(value)} Hz' if value.isdigit() else value

def strip_cells(series_list):
    """Per payload column: (series index, rate string) for every measured cell; the chart's
    commanded rate (the most common numeric rate, ties to the higher one) and the cells
    whose recorded rate differs from it."""
    cells={x:[] for x in EXPECTED_SIZES}
    for index,series in enumerate(series_list):
        for point in series['points']:
            cells[point['payload_bytes']].append((index,rate_of(point)))
    numeric=[value for column in cells.values() for _,value in column if value.isdigit()]
    base=max(set(numeric),key=lambda v:(numeric.count(v),int(v)))
    exceptions=[(x,index,value) for x,column in sorted(cells.items()) for index,value in column if value!=base]
    missing=[(x,index,cell['status']) for index,series in enumerate(series_list) for cell in series.get('missing_cells',[]) for x in [cell['payload_bytes']]]
    return cells,base,exceptions,missing

def exception_rows(series_list):
    cells,base,exceptions,missing=strip_cells(series_list)
    per_column={}
    for x,_,_ in exceptions+missing: per_column[x]=per_column.get(x,0)+1
    return max(per_column.values(),default=0)

def rate_strip(strip, series_list, colors, strip_pt, rows, mention_shortfalls=True, offered_note='', header_text=None):
    """Commanded vs achieved. A header states the commanded rate once. Each payload column
    carries one quiet confirmation (a light reference bar, labelled with the commanded rate)
    when every series achieved it; a series that did not gets its own coloured bar under the
    reference, its length the achieved fraction of the commanded rate, labelled with the
    achieved rate (or the CSV's word, 'mixed', as a dashed full-width bar). A cell that
    was not attempted is written as such in the series colour."""
    strip.set_ylim(0,1); strip.set_yticks([])
    for side in ('left','right','top'):
        strip.spines[side].set_visible(False)
    strip.spines['bottom'].set_color(FRAME)
    strip.set_facecolor(STRIP_BG)
    assert len(colors)==len(series_list)
    cells,base,exceptions,missing=strip_cells(series_list)
    assert rows==(exception_rows(series_list) if mention_shortfalls else 0)
    text_lines=2 if (missing and mention_shortfalls) else 1
    assert strip_pt==strip_height(rows,text_lines), (strip_pt,rows,text_lines)
    y=lambda pt:(strip_pt-pt)/strip_pt
    header=(header_text.format(rate=rate_text(base)) if header_text else
            ((f'Commanded rate: {rate_text(base)} on every cell.   Achieved rate: {rate_text(base)} confirmed in grey; '
              f'a shortfall is named in the series colour.') if mention_shortfalls else f'Commanded rate: {rate_text(base)} on every cell.')+offered_note)
    header_artist=strip.text(0.0,y(6+7),header,transform=strip.transAxes,ha='left',va='center',fontsize=SIZE['strip_header'],
               color=MUTED,clip_on=False)
    # The header must fit inside the strip box, or the offered-rate note spills past the canvas edge.
    canvas=strip.figure.canvas
    renderer=canvas.get_renderer() if hasattr(canvas,'get_renderer') else FigureCanvasAgg(strip.figure).get_renderer()
    text_w=header_artist.get_window_extent(renderer).width; axes_w=strip.get_window_extent(renderer).width
    assert text_w<=STRIP_HEADER_MAX_FRACTION*axes_w, f'strip header overflows its box: {text_w:.0f}px of {axes_w:.0f}px: {header!r}'
    label_top=6+14+6
    bar_top=label_top+13+4
    half=2**.42
    for x,column in cells.items():
        confirmed=[i for i,v in column if v==base] if mention_shortfalls else [i for i,_ in column]
        if confirmed:
            strip.text(x,y(label_top+6.5),rate_text(base),ha='center',va='center',fontsize=SIZE['strip'],color=MUTED,clip_on=False)
            strip.plot([x/half,x*half],[y(bar_top+2)]*2,color=STRIP_BAR,lw=3.2,solid_capstyle='butt',zorder=3)
        row=0
        entries=([(i,v,'measured') for i,v in column if v!=base]+[(i,s,'missing') for xx,i,s in missing if xx==x]) if mention_shortfalls else []
        for i,v,kind in entries:
            top=bar_top+4+4+row*(4+13*text_lines+4)
            if kind=='missing':
                strip.text(x,y(top+2+13),'not\nattempted',ha='center',va='center',fontsize=SIZE['strip'],
                           color=colors[i],alpha=.85,clip_on=False,linespacing=1.15)
            elif v.isdigit():
                frac=int(v)/int(base); assert 0<frac<1
                left=x/half; right=left*(x*half/left)**frac
                strip.plot([left,right],[y(top+2)]*2,color=colors[i],lw=3.2,solid_capstyle='butt',zorder=4)
                strip.text(x,y(top+4+2+6.5),rate_text(v),ha='center',va='center',fontsize=SIZE['strip'],
                           color=colors[i],fontweight='bold',clip_on=False)
            else:
                strip.plot([x/half,x*half],[y(top+2)]*2,color=colors[i],lw=3.2,linestyle=(0,(2.2,1.4)),
                           dash_capstyle='butt',zorder=4)
                strip.text(x,y(top+4+2+6.5),rate_text(v),ha='center',va='center',fontsize=SIZE['strip'],
                           color=colors[i],fontweight='bold',clip_on=False)
            row+=1
    return {
        'rate_column':'achieved_rate_hz',
        'commanded_label':rate_text(base),'shortfalls_shown':mention_shortfalls,
        'header_text':header,'header_fit_px':[round(text_w),round(axes_w)],
        'cells':[{'series':s['display_label'],'cells':[{'payload_bytes':p['payload_bytes'],'rate':rate_of(p)} for p in s['points']]}
                 for s in series_list],
        'exception_labels':[{'payload_bytes':x,'series':series_list[i]['display_label'],'rate':v} for x,i,v in exceptions],
        'not_attempted':[{'payload_bytes':x,'series':series_list[i]['display_label']} for x,i,_ in missing],
    }

def draw_key(fig, height_pt, key_top):
    """The mark key, one row under the series legend: p50 dot, p50 to p99 band."""
    def row(draw, label):
        area=DrawingArea(30,13,0,0)
        draw(area)
        text=TextArea(label,textprops=dict(fontsize=SIZE['key'],color=TEXT))
        return HPacker(children=[area,text],align='center',pad=0,sep=8)
    rows=[
        row(lambda a:a.add_artist(Circle((15,6.5),3.6,facecolor=KEY_INK,edgecolor='none')),'dot = p50'),
        row(lambda a:a.add_artist(Rectangle((2,2),26,9,facecolor=KEY_INK,alpha=.26,edgecolor='none')),'band = p50 to p99'),
    ]
    box=AnchoredOffsetbox(loc='upper left',child=HPacker(children=rows,align='center',pad=0,sep=22),
                          pad=.5,borderpad=0,frameon=True,bbox_to_anchor=(LEFT-.006,1-key_top/height_pt),
                          bbox_transform=fig.transFigure)
    box.patch.set(facecolor='white',edgecolor=FRAME,linewidth=.9,alpha=.97)
    box.patch.set_boxstyle('round,pad=0.55,rounding_size=0.6')
    box.set_gid('mark-key')
    fig.add_artist(box)
    return ['dot = p50','band = p50 to p99']

TITLE_NOTE='lower is better'
def heading(fig,height_pt,title,subtitle_lines):
    t=fig.text(LEFT,1-TITLE_TOP/height_pt,title,fontsize=SIZE['title'],fontweight='bold',ha='left',va='top')
    # the note sits after the title in the brand blue; placed from the layout face's width plus a 16 px gap
    canvas=fig.canvas; renderer=canvas.get_renderer() if hasattr(canvas,'get_renderer') else FigureCanvasAgg(fig).get_renderer()
    bb=t.get_window_extent(renderer); x=(bb.x1+16)/fig.bbox.width
    fig.text(x,1-TITLE_TOP/height_pt,TITLE_NOTE,fontsize=SIZE['title'],fontweight='bold',color=COLORS['multi'],ha='left',va='top',gid='title-note')
    for i,line in enumerate(subtitle_lines):
        fig.text(LEFT,1-(SUBTITLE_TOP+i*SUB_PITCH)/height_pt,line,fontsize=SIZE['subtitle'],color=MUTED,ha='left',va='top')

FOOT_WRAP = 132
STRIP_HEADER_MAX_FRACTION = 0.97   # header width over strip-box width; the SVG face may run a little wider than the layout face
NB_UNITS=r'(?<=\d) (?=(?:B|KiB|MiB|Hz|µs|ms|W|GHz|replies)\b)'
def nb(line):
    """Non-breaking spaces between a number and its unit, and inside 'ROS 2', so a wrap never splits them."""
    return re.sub(NB_UNITS,'\u00a0',line).replace('ROS 2','ROS\u00a02')
def footer(fig,height_pt,lines):
    wrapped=[]
    for line in lines:
        wrapped+=textwrap.wrap(nb(line),FOOT_WRAP,break_long_words=False,break_on_hyphens=False)
    for i,line in enumerate(wrapped):
        top=FOOT_LAST_TOP+FOOT_PITCH*(len(wrapped)-1-i)
        fig.text(LEFT,top/height_pt,line,fontsize=SIZE['footnote'],color=MUTED,ha='left',va='top')
    fig.text(RIGHT+.035,BRAND_Y/height_pt,'CERULION',fontsize=SIZE['brand'],fontweight='bold',color='#5D7491',ha='right',va='baseline')
    return wrapped

def wrapped_count(lines):
    return sum(len(textwrap.wrap(nb(l),FOOT_WRAP,break_long_words=False,break_on_hyphens=False)) for l in lines)

def series_legend(fig,height_pt,legend_top,handles,labels,ncol,columnspacing,row_offset=0):
    legend=fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(LEFT-.006,1-(legend_top+row_offset)/height_pt),ncol=ncol,
                      fontsize=SIZE['legend'],columnspacing=columnspacing,handlelength=2.6,labelspacing=.55,
                      handletextpad=.7,borderaxespad=0,borderpad=0)
    for text in legend.get_texts():
        if any(token in text.get_text() for token in CODE_TOKENS):
            text.set_fontfamily(LAYOUT_MONO)   # widest case for layout; the SVG pass sets only the token in mono
    return legend

def github_typography(svg):
    """Rewrite every font-family to the GitHub stacks and set code tokens in the mono stack."""
    text=svg.read_text(encoding='utf-8')
    text=re.sub(r'font-family: [^;"]*',f'font-family: {GITHUB_SANS}',text)
    def wrap(match):
        content=match.group(2)
        for token in CODE_TOKENS:
            content=content.replace(token,f'<tspan style="font-family: {GITHUB_MONO}; font-size: 0.9em">{token}</tspan>')
        return f'{match.group(1)}{content}</text>'
    text=re.sub(r'(<text [^>]*>)([^<]*)</text>',wrap,text)
    families=set(re.findall(r'font-family: ([^;"]*)',text))
    assert families<={GITHUB_SANS,GITHUB_MONO}, families
    for banned in ("'Helvetica Neue'","Menlo,", 'DejaVu','Inter','Menlo;'):
        assert banned not in text.replace(GITHUB_MONO,''), banned
    scrub_check(text)
    svg.write_text(text,encoding='utf-8')
    return sorted(families)

def uniquify_svg_ids(path,prefix):
    """Prefix every id in the SVG (and every url(#)/href="#" reference) with the figure name, so three
    figures inlined in one page cannot share an id; assert no duplicate id remains."""
    text=path.read_text()
    assert not any(i.startswith(prefix+'-') for i in re.findall(r'\bid="([^"]+)"',text))
    text=re.sub(r'\bid="([^"]+)"',lambda m:f'id="{prefix}-{m.group(1)}"',text)
    text=re.sub(r'url\(#([^)]+)\)',lambda m:f'url(#{prefix}-{m.group(1)})',text)
    text=re.sub(r'href="#([^"]+)"',lambda m:f'href="#{prefix}-{m.group(1)}"',text)
    ids=re.findall(r'\bid="([^"]+)"',text)
    dup=sorted({i for i in ids if ids.count(i)>1}); assert not dup, f'duplicate svg ids in {path.name}: {dup[:8]}'
    path.write_text(text)
    return len(ids)
def scrub_check(text):
    assert not re.search('[\u2013\u2014]',text), 'en or em dash in output'
    for prefix in ('/'+'Users'+'/', '/'+'home'+'/'):
        assert prefix not in text, 'home path in output'
    assert str(Path.home()) not in text and Path.home().name not in text, 'login name in output'

LEGEND_ALIAS={'ROS 2 composed + intra-process (one process, no isolation)':'ROS 2 composed + intra-process'}
def packet_view(series):
    """The exact dict shape the reviewed point hashes cover (the floor additions under 'v5' are
    left out); a legend text that only adds a note to a reviewed label hashes as the reviewed label,
    so the point comparison stays about points."""
    d={k:v for k,v in series.items() if k!='v5'}
    if d.get('display_label') in LEGEND_ALIAS: d['display_label']=LEGEND_ALIAS[d['display_label']]
    return d

def save(fig,name,series,notes,extra,added=(),added_info=None):
    all_series=list(series)+list(added)
    points=[p for s in all_series for p in s['points']]
    # Assert the actual plotted artists as well as their source data. A correct
    # manifest alone would not catch an accidental scale/series change here.
    data_axes=[ax for ax in fig.axes if str(ax.get_gid()).startswith('data-axes-')]
    labels={s['display_label'] for s in all_series}
    lines=[line for ax in data_axes for line in ax.lines if line.get_label() in labels]
    bands=[band for ax in data_axes for band in ax.collections]
    assert len(lines)==len(all_series)==len(bands)
    assert CURVES_DRAWN==[s['display_label'] for s in all_series]
    for k,(line,band,s) in enumerate(zip(lines,bands,all_series,strict=True)):
        x=[p['payload_bytes'] for p in s['points']]
        y=[us(p['p50_ns']) for p in s['points']]
        hi=[us(p['p99_ns']) for p in s['points']]
        assert list(line.get_xdata())==x and list(line.get_ydata())==y
        assert line.get_marker()=='o' and line.get_markerfacecolor()==line.get_color()
        vertices={tuple(pair) for pair in band.get_paths()[0].vertices}
        assert all((a,b) in vertices for a,b in zip(x,y,strict=True))
        assert all((a,b) in vertices for a,b in zip(x,hi,strict=True))
        assert len(s['v5']['floor'])==len(s['points']), (s['display_label'],len(s['v5']['floor']))
    metadata={'Title':notes['title'],'Description':notes['description'],
              'Creator':f'build_benchmark_plots.py; matplotlib {matplotlib.__version__}',
              'Date':None}
    svg=ASSETS/f'{name}.svg'; png=OUT/f'{name}.png'
    fig.savefig(svg,metadata=metadata)
    families=github_typography(svg)
    svg_ids=uniquify_svg_ids(svg,name)
    for legend in fig.legends:          # raster twin: one face per label, so uniform sans
        for text in legend.get_texts():
            text.set_fontfamily(LAYOUT_SANS)
    fig.savefig(png,dpi=160,metadata={'Software':f'matplotlib {matplotlib.__version__}'})
    packet=[packet_view(s) for s in series]
    points_sha=hashlib.sha256(json.dumps(packet,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    assert points_sha==packet_manifest['figures'][name]['points_sha256'], f'{name}: plotted points differ from the reviewed point hashes'
    floors=[{'series':s['display_label'],'floor':s['v5']['floor'],'floor_source':s['v5']['floor_source']} for s in all_series]
    floors_sha=hashlib.sha256(json.dumps(floors,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    entry={
        **notes,'series':packet,'point_count':len(points),
        'actual_artist_values_verified':True,'floor_bars_drawn':False,'floors_recorded_not_drawn':'floor bars removed from every figure; floors stay in the manifest as provenance','curve_count':len(lines),
        'points_sha256':points_sha,'points_sha256_equals_reviewed_hash':True,
        'floors':floors,'floors_sha256':floors_sha,
        'svg_sha256':sha(svg),'png_sha256':sha(png),
        'svg_font_families':families,'svg_id_prefix':name,'svg_id_count':svg_ids,'figure_size_in':[round(v,3) for v in fig.get_size_inches()],
        'render_css_px':[round(fig.get_figwidth()*72*4/3),round(fig.get_figheight()*72*4/3)],
        **extra,
    }
    if added:
        added_view=[packet_view(s) for s in added]
        assert added_info is not None, 'an added series must say where it came from'
        entry['added_series']={
            'series':added_view,
            'points_sha256':hashlib.sha256(json.dumps(added_view,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            **added_info,
        }
    manifest['figures'][name]=entry
    plt.close(fig)

# =====================================================================================
# Figure 1: the main round-trip chart, five native-campaign series plus rmw_cerulion fully loaned,
# all six measured by benches/latency in one harness, posture and pacing.
labels=['Cerulion multi-process (default)','Cerulion single-process','ROS 2 Jazzy / rmw_fastrtps_cpp','ROS 2 composed + intra-process (one process, no isolation)','zenoh shared memory']
colors=[COLORS['multi'],COLORS['single'],COLORS['stock'],COLORS['composed'],COLORS['zenoh']]
styles=[DASH['solid'],DASH['dashed'],DASH['solid'],DASH['dashdot'],DASH['dotted']]
rmw_on_main={**rmw_same,'display_label':'rmw_cerulion'}
main_series=native+[rmw_on_main]; main_colors=colors+[COLORS['rmw']]; main_styles=styles+[DASH['shortdash']]
native_date=date_of(HEROES); rmw_same_date=date_of(RMW_SAME)
subtitle=['Same-host comparison on an x86-64 Linux desktop (24-core Intel Core Ultra 9 285K)',
          'STOCK  ·  recorder ON  ·  fixed 100 Hz  ·  k=5']
COUNT_WORDS={5:'five'}
foot=[
    'Commanded 100 Hz, achieved on every cell by the Cerulion and zenoh lanes; the two ROS 2 lanes record the rate offered, not what came back: at 16 MiB a stock ROS 2 round trip outlasts the 10 ms pacing interval, so requests overlap in flight. Native Cerulion includes always-on recording; the ROS 2 and zenoh runs do not. Payload fill excluded.',
    'rmw_cerulion is ROS 2 Jazzy over Cerulion\'s transport, measured in the same harness, posture and pacing as the other lines: three nodes in three processes with full process isolation. Non-POD, variable-length ROS messages work too, with no code to rewrite.',
    'ROS 2 composed + intra-process runs every node in one process: lower latency by giving up crash isolation, and its timed path bypasses the middleware, so outside tools do not see the traffic being measured.',
    'Five sessions per point. p50: median of the five session medians; p99: 10,000 pooled samples.',
]
# The typed counts in the last line hold for every series drawn, the rmw_cerulion row included.
assert all((p['repetitions'],p['samples'])==(5,10000) for s in main_series for p in s['points'])
strip_pt=0   # the rate strip is retired
legend_top,key_top,top_pt=header_height(len(subtitle),legend_rows=3)
fig,ax,strip,height_pt=make_figure(top_pt=top_pt,main_pt=300,strip_pt=strip_pt,bottom_pt=bottom_height(wrapped_count(foot)))
heading(fig,height_pt,'Round-trip latency across message sizes',subtitle)
for i,(series,label,color,style) in enumerate(zip(main_series,labels+[rmw_on_main['display_label']],main_colors,main_styles,strict=True)):
    series['display_label']=label
    curve(ax,series,label,color,linestyle=style,zorder=10-i)
payload_axis(ax,strip);latency_axis(ax,1,300000,[1,10,100,1000,10000,100000])
ends=end_labels(ax,[(us(s['points'][-1]['p50_ns']),c,fmt_value(s['points'][-1]['p50_ns'])) for s,c in zip(main_series,main_colors,strict=True)])
series_legend(fig,height_pt,legend_top,*ax.get_legend_handles_labels(),ncol=2,columnspacing=2.2)
key=draw_key(fig,height_pt,key_top)
foot_lines=footer(fig,height_pt,foot)
save(fig,'native-rtt',native,{'title':'Round-trip latency across message sizes',
    'description':'Six measured series from one harness, posture and pacing: the five series of the native fixed 100 Hz campaign plus ROS 2 Jazzy over rmw_cerulion, three nodes in three processes with full process isolation. Commanded 100 Hz; the footnote carries the offered-rate caveat for the stock and composed ROS 2 lanes.',
    'rate_semantics':'Native stock ROS2 sidecar 100 is nominal offered, not a verified 100 Hz delivered rate; the footnote says so. At 16 MiB the p50 round trip of the stock lane in the package CSV exceeds the 10 ms pacing interval, so its requests overlap in flight.',
    'percentiles':'all six series: p50 median of five per-session p50s, p99 pooled 10000 samples (compile_csv.py, linear interpolation truncated to a whole nanosecond)',
    'capture':'Native Cerulion recorder on; the ROS 2 runs (rmw_cerulion included) and direct zenoh have no native graph recorder.',
    'rmw_cerulion_rate':'achieved_rate_hz 100 on all ten rows, read from the CSV and from all 50 .rate sidecars (asserted)',
    'package_dates':{'native_campaign':native_date,'rmw_cerulion_same_harness':rmw_same_date},
    'subtitle':subtitle,'footnote_lines':foot_lines,
},{'mark_key':key,'end_labels':ends,'line_styles':dict(zip(labels+[rmw_on_main['display_label']],[str(s) for s in main_styles]))},
    added=[rmw_on_main],added_info={
        'source':rmw_same['source'],'source_sha256':sha(SOURCE/rmw_same['source']),
        'point_ledger':{'path':rel(RMW_SAME_LEDGER),'sha256':sha(RMW_SAME_LEDGER)},
        **rmw_same_meta})

# =====================================================================================
# Figure 2: one wide panel, six series; colour = platform, line style = run shape.
platform_names=['x86-64 desktop (Core Ultra 9 285K)','Jetson Orin NX (Linux aarch64)','Apple M4 laptop (macOS)']
platform_short=['x86-64','Jetson','Mac']
platform_colors=[COLORS['multi'],COLORS['jetson'],COLORS['mac']]
six_colors=[c for c in platform_colors for _ in (0,1)]
six_styles=[DASH['solid'],DASH['dashed']]*3
for index,(split,mono) in enumerate(platforms[i*2:i*2+2] for i in range(3)):
    split['display_label']='Multi-process (default)';mono['display_label']='Single-process'
x86_split,x86_mono,jet_split,jet_mono,mac_split,mac_mono=platforms
def p50s(s): return [p['p50_ns'] for p in s['points']]
jet_elev=[a-b for s in (jet_split,jet_mono) for a,b in zip(p50s(s),jetson_floor,strict=True)]
x86_elev=[a-b for s in (x86_split,x86_mono) for a,b in zip(p50s(s),x86_floor,strict=True)]
mixed_cells=[(SIZE_LABELS[EXPECTED_SIZES.index(x)],platform_short[i//2],platforms[i]['display_label']) for x,i,v in strip_cells(platforms)[2]]
assert all(v=='mixed' for _,_,v in strip_cells(platforms)[2])
jet_date=date_of(JETSON); mac_date=date_of(MAC); assert jet_date==mac_date
subtitle=['Same payload sweep on three machines  ·  colour = platform, line style = run shape',
          'STOCK  ·  recorder ON  ·  fixed 100 Hz  ·  k=5']
def signed_range(values):
    lo,hi=us(min(values)),us(max(values))
    if lo<0: return f'from {-lo:.1f} µs below to {hi:.1f} µs above'
    return f'{lo:.1f} to {hi:.1f} µs above'
foot=[
    'Commanded 100 Hz; five sessions, 10,000 samples per point. Always-on recording included. Payload fill excluded.',
    'Jetson: NV Power Mode 40 W, governor schedutil, CPU clock capped at 1.4976 GHz (hardware max 1.984 GHz).',
    f'Mac: multi-process on macOS waits on a bounded spin plus a sleep-and-recheck, so its round trip sits near {us(sorted(p50s(mac_split))[5]):.0f} µs ({fmt_range_us(p50s(mac_split))} p50); '
    f'single-process ({fmt_range_us(p50s(mac_mono))} p50) is the representative Mac number.',
]
strip_pt=0
legend_top,key_top,top_pt=header_height(len(subtitle),legend_rows=2)
fig,ax,strip,height_pt=make_figure(top_pt=top_pt,main_pt=300,strip_pt=strip_pt,bottom_pt=bottom_height(wrapped_count(foot)))
heading(fig,height_pt,'Native Cerulion on three platforms',subtitle)
for i,(series,color,style) in enumerate(zip(platforms,six_colors,six_styles,strict=True)):
    curve(ax,series,series['display_label'],color,linestyle=style,zorder=10-i)
pl_floor=min(us(f['floor_ns']) for s_ in platforms for f in s_['v5']['floor'])
ymin_pl=1.5 if pl_floor>=1.7 else 1
payload_axis(ax,strip);latency_axis(ax,ymin_pl,300,[t for t in (1,2,3,10,30,100,300) if t>=ymin_pl])
xm=EXPECTED_SIZES[4]; ym=us(p50s(mac_split)[4]); ytop=us(max(p['p99_ns'] for p in mac_split['points']))
ax.annotate('lower Mac latencies coming soon',xy=(xm,ym),xytext=(xm,ytop*1.35),ha='center',va='bottom',
            fontsize=SIZE['footnote']+1,color=COLORS['mac'],fontweight='bold',
            arrowprops=dict(arrowstyle='-',color=COLORS['mac'],lw=.9,alpha=.7,shrinkB=4),zorder=20)
ends=end_labels(ax,[(us(s['points'][-1]['p50_ns']),c,fmt_value(s['points'][-1]['p50_ns'])) for s,c in zip(platforms,six_colors,strict=True)])
platform_handles=[Line2D([],[],color=c,lw=2.2,marker='o',markersize=6.4,markerfacecolor=c,markeredgecolor='white',markeredgewidth=.9) for c in platform_colors]
shape_handles=[Line2D([],[],color=KEY_INK,lw=2.2,linestyle=s) for s in (DASH['solid'],DASH['dashed'])]
series_legend(fig,height_pt,legend_top,platform_handles,platform_names,ncol=3,columnspacing=2.2)
series_legend(fig,height_pt,legend_top,shape_handles,['Multi-process (default)','Single-process'],ncol=2,columnspacing=2.2,row_offset=LEGEND_ROW_PT)
key=draw_key(fig,height_pt,key_top)
foot_lines=footer(fig,height_pt,foot)
save(fig,'platform-rtt',platforms,{'title':'Native Cerulion on three platforms',
    'description':'One panel, six series: multi-process (solid) and single-process (dashed) on Linux x86-64 (blue), Jetson Orin NX (orange) and Apple M4 (violet). Recorder enabled.',
    'percentiles':'p50 median of five per-session medians; p99 pooled 10000 samples',
    'capture':'Native rolling recorder enabled on all shown native cells; no recorder-off causal comparison.',
    'platform_of_series':[{'series':i,'platform':platform_names[i//2],'shape':platforms[i]['display_label'],'colour':six_colors[i],'style':str(six_styles[i])} for i in range(6)],
    'footnote_numbers_read':{'jetson_floor_p50_ns':jetson_floor,'x86_floor_p50_ns':x86_floor,'jetson_elevation_ns':jet_elev,'x86_elevation_ns':x86_elev,
                             'mac_split_p50_ns':p50s(mac_split),'mac_mono_p50_ns':p50s(mac_mono),'mixed_cells':mixed_cells,
                             'jetson_machine_configuration_source':'the footnote: machine configuration recorded after the campaign, not a measured latency'},
    'subtitle':subtitle,'footnote_lines':foot_lines,
},{'mark_key':key,'end_labels':ends})

manifest['renderer_sha256']=sha(Path(__file__))
text=json.dumps(manifest,indent=2,ensure_ascii=False)+'\n'
scrub_check(text)
(OUT/'benchmark-plot-manifest.json').write_text(text)
print(f'Wrote docs/media/native-rtt.svg and docs/media/platform-rtt.svg (PNG twins and the manifest under docs/media/charts/out/); {sum(f["point_count"] for f in manifest["figures"].values())} plotted p50/p99 pairs verified; floors recorded in the manifest, not drawn.')
print('Every source CSV equals the point ledger, and every figure point hash equals reviewed-point-hashes.json.')
print(f'Main chart rmw_cerulion series: same-harness package, 10 points equal the point ledger, {rmw_same_meta["raw_rereduction"]["files"]} raw files ({rmw_same_meta["raw_rereduction"]["samples"]} samples) re-reduced to the CSV, posture equals the stock ROS 2 invocations.')
