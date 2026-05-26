# Artifacts Layout

All generated simulation outputs should stay under this directory.

## Structure

```text
artifacts/
└── line_follow_v1/
    ├── bin/       # compiled runners
    ├── logs/      # optional runtime logs
    └── reports/
        ├── single_run/
        ├── stability_runs/
        ├── regression/
        └── baseline/
```

## Rule

- New scripts and tools must write outputs to `artifacts/`.
- Do not place generated reports back into firmware source directories.
