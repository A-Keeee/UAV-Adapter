"""Select latest attempts; a historical pass never hides a newer failure."""


def latest_configurations(records):
    latest = {}
    for record in records:
        key = (record['task'], record['simulated_payload'], record['condition'])
        if key not in latest or int(record.get('attempt', 1)) >= int(latest[key].get('attempt', 1)):
            latest[key] = record
    return latest


def latest_fault_configurations(records):
    latest = {}
    for record in records:
        key = (record['fault_kind'], record['simulated_payload'])
        if key not in latest or int(record.get('attempt', 1)) >= int(latest[key].get('attempt', 1)):
            latest[key] = record
    return latest
