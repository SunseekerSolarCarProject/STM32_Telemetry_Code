# RTC, GPS UTC, and local time in Telemetry Advanced V2

## Time basis

The PCF85263A stores UTC. A valid NMEA RMC sentence supplies UTC time and date;
the board never stores Eastern, Central, daylight, or standard local time in
the hardware calendar.

All formatted RTC timestamps end in `Z`, for example:

```text
2026-07-22T16:43:29Z
```

The `Z` is important: it tells the telemetry application that the value is UTC
and prevents a local-time conversion from being applied in the wrong direction.

## Synchronization

- The first checksum-valid active RMC sentence checks and, if necessary, sets
  the PCF85263A.
- While GPS remains available, the board verifies the RTC once per minute.
- A difference greater than two seconds causes the RTC to be rewritten.
- `SET_RTC,YYYY-MM-DD,HH:MM:SS` is interpreted as UTC.
- After `SET_RTC`, the next valid RMC is forced to verify or correct the value
  immediately. A manually supplied local wall-clock value therefore cannot
  remain in the RTC until the next periodic check.

## Local timezone display

GPS provides UTC and coordinates, not an IANA timezone name or daylight-saving
rules. V2 sends `rtc_basis=UTC`, GPS latitude, GPS longitude, and GPS-fix age in
the `$TEL` status frame. The application should:

1. Parse `rtc` as an absolute UTC timestamp because it ends in `Z`.
2. Resolve the fresh GPS coordinates to an IANA zone such as
   `America/New_York` or `America/Chicago`.
3. Convert UTC to that zone using the operating system's current timezone
   database.
4. Display both values when useful, such as
   `12:43:29 EDT / 16:43:29 UTC`.

The application must not subtract the local offset before sending `SET_RTC`.
If it sends a manual value, it must send the current UTC calendar value.

## Example corresponding to the reported capture

For:

```text
2026-07-22 12:43:29 Eastern Daylight Time
```

the board RTC and wire timestamp should be:

```text
2026-07-22T16:43:29Z
```

The application performs the UTC-to-Eastern conversion. The PCF85263A remains
unchanged when the vehicle crosses a timezone boundary or daylight-saving
transition.
