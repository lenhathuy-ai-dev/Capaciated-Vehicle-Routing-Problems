            ref = f'{excel_col(c)}{r}'
            style = 1 if r == 1 else (2 if r % 2 == 0 else 0)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                out.append(f'<c r="{ref}" s="{style}"><v>{v}</v></c>')
            else:
                out.append(f'<c r="{ref}" s="{style}" t="inlineStr"><is><t>{esc(v)}</t></is></c>')
        out.append('</row>')
    out.append('</sheetData>')
    if len(rows) > 1:
        out.append(f'<autoFilter ref="A1:{last_col}{last_row}"/>')
    out.append('</worksheet>')
    return ''.join(out)


def styles_xml():
    return ('<?xml version="1.0" encoding="utf-8"?>'
            '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
            '<fonts count="2"><font><sz val="11"/><name val="Aptos"/></font>'
            '<font><b/><sz val="11"/><color rgb="FFFFFFFF"/><name val="Aptos"/></font></fonts>'
            '<fills count="4"><fill><patternFill patternType="none"/></fill>'
            '<fill><patternFill patternType="gray125"/></fill>'
            '<fill><patternFill patternType="solid"><fgColor rgb="FF17365D"/></patternFill></fill>'
            '<fill><patternFill patternType="solid"><fgColor rgb="FFF0F5FA"/></patternFill></fill>'
            '</fills><borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>'
            '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
            '<cellXfs count="3"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
            '<xf numFmtId="0" fontId="1" fillId="2" borderId="0" xfId="0" applyFont="1" applyFill="1"/>'
            '<xf numFmtId="0" fontId="0" fillId="3" borderId="0" xfId="0" applyFill="1"/></cellXfs>'
            '<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>'
            '</styleSheet>')


def save_xlsx(target, sheets):
    ns = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
    relroot = 'http://schemas.openxmlformats.org/package/2006/relationships'
    book_sheets = ''.join(f'<sheet name="{esc(name)}" sheetId="{i}" r:id="rId{i}"/>'
                          for i, (name, _, _) in enumerate(sheets, 1))
    book = ('<?xml version="1.0" encoding="utf-8"?>'
            f'<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="{ns}">'
            f'<sheets>{book_sheets}</sheets></workbook>')
    rels = ''.join(f'<Relationship Id="rId{i}" Type="{ns}/worksheet" Target="worksheets/sheet{i}.xml"/>'
                   for i in range(1, len(sheets) + 1))
    rels += f'<Relationship Id="rId{len(sheets)+1}" Type="{ns}/styles" Target="styles.xml"/>'
    types = ('<?xml version="1.0" encoding="utf-8"?>'
             '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
             '<Default Extension="xml" ContentType="application/xml"/>'
             '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
             '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
             '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>')
    for i in range(1, len(sheets) + 1):
        types += (f'<Override PartName="/xl/worksheets/sheet{i}.xml"'
                  ' ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>')
    types += '</Types>'
    with ZipFile(target, 'w', ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', types)
        z.writestr('_rels/.rels',
                   f'<?xml version="1.0"?><Relationships xmlns="{relroot}">'
                   f'<Relationship Id="rId1" Type="{ns}/officeDocument" Target="xl/workbook.xml"/>'
                   '</Relationships>')
        z.writestr('xl/workbook.xml', book)
        z.writestr('xl/_rels/workbook.xml.rels', f'<Relationships xmlns="{relroot}">{rels}</Relationships>')
        z.writestr('xl/styles.xml', styles_xml())
        for i, (_, rows, widths) in enumerate(sheets, 1):
            z.writestr(f'xl/worksheets/sheet{i}.xml', sheet_xml(rows, widths))


def main():
    ap = argparse.ArgumentParser(description='Export CVRP Optimal/BKS reference values into Excel sheets.')
    ap.add_argument('--data', type=Path, default=DEFAULT_DATA, help='Benchmark root folder')
    ap.add_argument('--offline', action='store_true', help='Do not connect to CVRPLIB')
    args = ap.parse_args()
    root = args.data
    if not root.is_dir():
        raise SystemExit(f'Khong tim thay thu muc: {root}')
    refs = load_reference(root, args.offline)
    candidates = sorted((p for p in root.rglob('*') if p.is_file() and p.suffix.lower() in SUFFIXES),
                        key=lambda p: natural_key(str(p.relative_to(root))))
    by_dataset = defaultdict(list)
    seen = set()
    for p in candidates:
        # Prioritize VRP over TXT and XML when duplicates represent same instance.
        try:
            record = parse_instance(p, root, refs)
            if not record:
                continue
            distinct = (record['dataset'].casefold(), keyname(record['name']))
            if distinct in seen:
                continue
            seen.add(distinct)
            by_dataset[record['dataset']].append(record)
        except (OSError, ValueError, UnicodeError) as ex:
            print(f'[BO QUA] {p}: {ex}')
    if not by_dataset:
        raise SystemExit('Khong co file benchmark .vrp/.txt/.xml. Hay giai nen .7z truoc.')
    order = sorted(by_dataset, key=lambda d: (SETS.index(d) if d in SETS else len(SETS), d.lower()))
    summary = [['Dataset', 'Tong instance', 'Optimal', 'BKS', 'SOL', 'Chua co gia tri']]
    sheets = []
    for ds in order:
        items = sorted(by_dataset[ds], key=lambda r: natural_key(r['name']))
        counts = Counter(v['kind'] for v in items)
        summary.append([ds, len(items), counts['Optimal'], counts['BKS'], counts['SOL'], counts['Chua co gia tri']])
        rows = [HEADERS]
        for i, r in enumerate(items, 1):
            rows.append([i, r['name'], r['customers'], r['trucks'], r['capacity'],
                         r['value'], r['kind'], r['source'], r['url'], r['path'], r['note']])
        sheets.append((ds, rows, [7, 27, 17, 12, 17, 22, 20, 31, 65, 90, 70]))
    sheets.insert(0, ('TONG_HOP', summary, [22, 20, 17, 17, 17, 22]))
    target = root / 'CVRP_Optimal_BKS_by_Dataset.xlsx'
    try:
        save_xlsx(target, sheets)
    except PermissionError:
        raise SystemExit('Khong ghi duoc Excel. Hay dong file Excel cu roi chay lai.')
    print('\n===== HOAN THANH =====')
    print(f'Excel: {target}')
    print(f'Tong: {sum(len(items) for items in by_dataset.values())} instance, {len(order)} dataset.')
    print('Sheets:', ', '.join(name for name, _, _ in sheets))
    print('Optimal: gia tri da duoc xac nhan; BKS: can tren tot nhat hien biet; SOL: chi phi file nghiem.')
    print('Chu y: code KHONG tu giai bai toan CVRP.')


if __name__ == '__main__':
    main()
