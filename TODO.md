# TODO: Enhanced Flight Review

The current `analysis/review_runs.py` script only summarizes logs and ensures
`flight_view` HTML files exist. We want a more comprehensive review that mirrors
the written analysis style seen in recent chat transcripts. Future flight
reviews should automatically:

1. Parse the latest `full_log_*.csv` files and compute statistics such as frame
   count, collision count, average FPS, and traveled distance.
2. Load the generated `flight_view_*.html` files for each log (or generate them
   if missing) and inspect the UAV path in relation to obstacles from
   `analysis/obstacles.json`.
3. Calculate the path start and end points relative to `PlayerStart_3` and note
   any obstacles (e.g., `Cube11`) the UAV fails to reach or passes.
4. Highlight repeated states like `brake`, `resume`, and whether `dodge` events
   occur. Identify where the UAV stalls or collides.
5. Output a concise report combining these findings so that when we request a
   "flight review" it includes:
   - Log statistics table
   - FPS and loop time summary
   - Path versus obstacle commentary similar to the previous example response
   - Suggestions for improving obstacle avoidance

Implement these capabilities either by extending `review_runs.py` or creating a
new `analysis/flight_review.py` module. Tests should cover log parsing and
obstacle alignment logic.
