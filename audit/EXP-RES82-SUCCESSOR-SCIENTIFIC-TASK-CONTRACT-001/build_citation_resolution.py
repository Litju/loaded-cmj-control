#!/usr/bin/env python3
"""Resolve every external evidence-table citation against Europe PMC.

Writes citation_resolution.json with the exact query, resolved metadata, response
digest, and status. Network-required; run once and archive the output.
"""
from __future__ import annotations

import hashlib
import json
import urllib.parse
import urllib.request
from datetime import datetime, timezone

SOURCES = [
    ("S01", "EXT_ID:39425876", "39425876", "10.1007/s40279-024-02098-x"),
    ("S02", "EXT_ID:41672931", "41672931", "10.1002/ejsc.70114"),
    ("S03", "EXT_ID:36940054", "36940054", "10.1007/s40279-023-01828-x"),
    ("S04", "EXT_ID:38863789", "38863789", "10.70252/QBUA4521"),
    ("S05", "EXT_ID:36548490", "36548490", "10.3390/sports10120193"),
    ("S06", "EXT_ID:31711369", "31711369", "10.1080/14763141.2019.1682649"),
    ("S07", "EXT_ID:28632047", "28632047", "10.1080/14763141.2016.1246598"),
    ("S08", "EXT_ID:20029097", "20029097", "10.1123/ijspp.4.4.461"),
    ("S09", "EXT_ID:22209596", "22209596", "10.1016/j.jelekin.2011.12.002"),
    ("S10", "EXT_ID:24875041", "24875041", "10.1080/02640414.2014.924055"),
    ("S11", "EXT_ID:41295762", "41295762", "10.3390/sports13110379"),
    ("S12", "EXT_ID:41465769", "41465769", "10.3390/life15121830"),
    ("S13", "EXT_ID:36676138", "36676138", "10.3390/life13010190"),
    ("S14", "EXT_ID:37815253", "37815253", "10.1519/JSC.0000000000004586"),
    ("S15", "EXT_ID:25243113", "25243113", "10.1155/2014/126860"),
    ("S16", "EXT_ID:30902539", "30902539", "10.1016/j.jsams.2019.03.001"),
    ("S17", "EXT_ID:18037546", "18037546", "10.1016/j.clinbiomech.2007.10.003"),
    ("S18", "EXT_ID:19295962", "19295962", "10.4085/1062-6050-44.2.174"),
    ("S19", "EXT_ID:41017646", "41017646", "10.1177/19417381251372976"),
    ("S20", "EXT_ID:16404451", "16404451", None),
    ("S21", "EXT_ID:17473764", "17473764", "10.1249/mss.0b013e31802d3460"),
    ("S22", "EXT_ID:9214811", "9214811", "10.1109/10.532130"),
    ("S23", "EXT_ID:15519333", "15519333", "10.1016/j.jbiomech.2004.03.025"),
]

def query(q: str):
    url = ("https://www.ebi.ac.uk/europepmc/webservices/rest/search?query="
           + urllib.parse.quote(q) + "&resultType=core&format=json&pageSize=1")
    with urllib.request.urlopen(url, timeout=45) as r:
        raw = r.read()
    return raw, json.loads(raw)

def main() -> None:
    out = {
        "artifact": "RES82-CITATION-RESOLUTION",
        "method": "Europe PMC REST search by PMID (EXT_ID) with DOI cross-check",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "sources": [],
        "non_epmc": [
            {"SOURCE_ID": "S24", "IDENTIFIER": "ASME V&V 40-2018; ISBN 9780791872048",
             "VERIFICATION": "ASME store listing (asme.org) confirmed title, publisher, 2018 edition on 2026-09-13"},
            {"SOURCE_ID": "S25", "IDENTIFIER": "FDA guidance https://www.fda.gov/media/154985/download",
             "VERIFICATION": "FDA guidance page confirmed final guidance issued 2023-11-16 on 2026-09-13"}
        ],
    }
    for sid, q, pmid, doi in SOURCES:
        entry = {"SOURCE_ID": sid, "QUERY": q, "EXPECTED_PMID": pmid, "EXPECTED_DOI": doi}
        try:
            raw, data = query(q)
            res = data.get("resultList", {}).get("result", [])
            if not res:
                entry["STATUS"] = "NOT_FOUND"
            else:
                x = res[0]
                entry.update({
                    "STATUS": "RESOLVED",
                    "PMID": x.get("pmid"),
                    "DOI": x.get("doi"),
                    "TITLE": x.get("title"),
                    "JOURNAL": x.get("journalInfo", {}).get("journal", {}).get("title"),
                    "YEAR": x.get("pubYear"),
                    "AUTHORS": x.get("authorString"),
                    "PMID_MATCH": x.get("pmid") == pmid,
                    "DOI_MATCH": (x.get("doi") or "").lower() == (doi or "").lower() if doi else None,
                })
            entry["RESPONSE_SHA256"] = hashlib.sha256(raw).hexdigest()
        except Exception as exc:  # noqa: BLE001
            entry["STATUS"] = "ERROR"
            entry["ERROR"] = str(exc)
        out["sources"].append(entry)
    resolved = sum(1 for s in out["sources"] if s.get("STATUS") == "RESOLVED")
    mismatch = [s["SOURCE_ID"] for s in out["sources"]
                if s.get("STATUS") == "RESOLVED" and (s.get("PMID_MATCH") is False or s.get("DOI_MATCH") is False)]
    out["summary"] = {"resolved": resolved, "total": len(out["sources"]), "identifier_mismatches": mismatch}
    with open("citation_resolution.json", "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps(out["summary"], indent=2))

if __name__ == "__main__":
    main()
