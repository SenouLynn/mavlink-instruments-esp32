CXX ?= c++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -pedantic -Ilib/hud-core/include
CORE := lib/hud-core/src/logic.cpp lib/hud-core/src/scene.cpp
BUILD := build

.PHONY: test preview clean

test: $(BUILD)/hud-tests
	./$(BUILD)/hud-tests

preview: $(BUILD)/hud-preview
	./$(BUILD)/hud-preview > $(BUILD)/hud-preview.svg
	@echo "Wrote $(BUILD)/hud-preview.svg"

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/hud-tests: test/hud_tests.cpp $(CORE) | $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/hud-preview: native/hud_preview.cpp $(CORE) | $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD)

