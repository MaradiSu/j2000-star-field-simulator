"""Development checks only. Usage: python tests/verify.py path/to/star_sim"""
from pathlib import Path
import csv, hashlib, math, struct, subprocess, sys, uuid

def check(condition, message):
    if not condition: raise RuntimeError(message)

def main():
    check(len(sys.argv)==2,'Provide the compiled executable path')
    exe=Path(sys.argv[1]).resolve()
    base=Path(__file__).resolve().parent.parent
    def run(*args):return subprocess.run([str(exe),*(str(a) for a in args)],capture_output=True,text=True)
    p=run('--self-test');check(p.returncode==0,p.stderr or 'Self-test failed')
    print(p.stdout.strip())
    work=Path.cwd()/('jsfs_check_'+uuid.uuid4().hex[:12])
    work.mkdir()
    if work.is_dir():
        for name in ['a','b']:
            d=work/name;d.mkdir()
            p=run(base/'catalogue_j2000.csv',base/'attitude.csv',base/'camera.csv',str(d)+'/')
            check(p.returncode==0,p.stderr or 'Scenario failed')
        rows=list(csv.DictReader((work/'a/spots.csv').open()));check(len(rows)==35,'Expected 35 spot rows')
        by={(int(r['frame']),r['id']):r for r in rows};fx=256/math.tan(math.radians(10))
        for k in range(5):
            r=by[k,'DEMO_CENTER']
            check(abs(float(r['u_px'])-(255.5-fx*math.tan(math.radians(k*.008))))<1e-8,'Analytic motion mismatch')
            check(abs(float(r['v_px'])-255.5)<1e-8,'Vertical center mismatch')
            with (work/f'a/frame_{k:04d}.pgm').open('rb') as f:
                check(f.readline()==b'P5\n','PGM magic')
                check(f.readline()==b'512 512\n','PGM dimensions')
                check(f.readline()==b'65535\n','PGM range')
                raw=f.read();check(len(raw)==524288,'PGM byte count')
                pixels=struct.unpack('>262144H',raw)
                check(max(pixels)>10000 and min(pixels)==0,'PGM content')
        check(abs(float(by[0,'DEMO_RIGHT']['u_px'])-(255.5+fx*math.tan(math.radians(5))))<1e-8,'Right star mismatch')
        commands=list(csv.DictReader((work/'a/mock_stos_commands.csv').open()))
        check(len(commands)==5,'Command count')
        for r in commands:check(abs(sum(float(r[x])**2 for x in ['qSI_w','qSI_x','qSI_y','qSI_z'])-1)<1e-12,'Quaternion norm')
        bad=work/'bad.csv';bad.write_text('time_s,qBI_w,qBI_x,qBI_y,qBI_z\n0,0,0,0,0\n')
        p=run(base/'catalogue_j2000.csv',bad,base/'camera.csv',str(work/'bad_'))
        check(p.returncode==1 and 'Invalid quaternion' in p.stderr,'Expected invalid quaternion rejection')
        for f in (work/'a').iterdir():check(f.read_bytes()==(work/'b'/f.name).read_bytes(),'Repeat output mismatch')
    print('PASS limited analytic, file, invalid quaternion and repeat-run checks')
    print('Results retained at',work)
    print('Executable SHA256',hashlib.sha256(exe.read_bytes()).hexdigest())
    print('No target timing, hardware qualification or coverage claim')

if __name__=='__main__':main()

