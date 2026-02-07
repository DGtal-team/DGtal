#!/bin/bash

WL=""
for i in $(git diff --name-only HEAD HEAD~5 | grep '\.cpp$'); do
    echo -n "${WL}$(basename "$i" .cpp)"
    WL=";"
done
