# Board Layout

Each hardware board keeps its own CubeMX-generated project under `Board/<Name>/`.

Current rule:
- `Board/<Name>/Core`, `Drivers`, `Middlewares`, `MDK-ARM`, and the `.ioc` file stay board-local.
- Shared device APIs stay under `User/Devices/`.
- Board-specific device mappings live in `Board/<Name>/Config/`.

## Adding A New Board

Create these files for the new board:

- `Board/<Name>/Config/dev_config_board.h`
- `Board/<Name>/Config/dev_config_board.inc`

Then:

1. Define the board enable macros in `dev_config_board.h`.
2. Fill the `*_list` tables in `dev_config_board.inc`.
3. Add the board macro to the MDK target, for example `JM_BOARD_<NAME>`.
4. Add the `board_select.h` branch for the new board.

## Existing Boards

- `Board/V1/`
- `Board/SFOC/` as a git submodule
