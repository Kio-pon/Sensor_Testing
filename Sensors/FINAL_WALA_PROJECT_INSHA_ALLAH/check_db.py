import sqlite3
import glob

dbs = glob.glob(r'C:\Users\Student\.gemini\antigravity-ide\conversations\*.db')
for db_file in dbs:
    try:
        conn = sqlite3.connect(db_file)
        c = conn.cursor()
        c.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = c.fetchall()
        for t in tables:
            table_name = t[0]
            try:
                c.execute(f"SELECT * FROM {table_name} LIMIT 1")
                columns = [desc[0] for desc in c.description]
                if 'id' in columns and 'title' in columns:
                    c.execute(f"SELECT id, title FROM {table_name}")
                    rows = c.fetchall()
                    for r in rows:
                        if 'a0644381' in str(r[0]):
                            print('FOUND:', r)
            except Exception:
                pass
        conn.close()
    except Exception:
        pass
