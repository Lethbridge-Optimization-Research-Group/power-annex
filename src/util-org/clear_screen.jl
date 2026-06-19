# Defines quick functionality for clearing the terminal after messy outputs.

"Clears the full terminal screen of all text and history"
function clear()
    print("\ec")
end

macro clear()
    print("\ec")
end
