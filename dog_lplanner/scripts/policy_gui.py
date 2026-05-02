#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tkinter GUI: toggles /policy_running and /policy_safe_mode for policy_inference_ros1.
"""

import rospy
import tkinter as tk
from tkinter import ttk


class PolicyGUI:
    def __init__(self):
        rospy.init_node('policy_gui', anonymous=True)

        self.root = tk.Tk()
        self.root.title("Policy Control")
        self.root.geometry("380x210")
        self.root.resizable(False, False)
        
        if not rospy.has_param('/policy_running'):
            rospy.set_param('/policy_running', False)
        if not rospy.has_param('/policy_safe_mode'):
            rospy.set_param('/policy_safe_mode', True)
        
        self.is_running = rospy.get_param('/policy_running', False)
        self.is_safe_mode = rospy.get_param('/policy_safe_mode', True)
        
        self.create_ui()
        self.update_ui()
        self.root.after(100, self.update_state)

    def create_ui(self):
        title_label = tk.Label(
            self.root,
            text="Policy Control",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        self.status_label = tk.Label(
            self.root,
            text="Status: STOPPED",
            font=("Arial", 12),
            fg="red"
        )
        self.status_label.pack(pady=5)

        # Safe mode toggle
        self.safe_var = tk.BooleanVar(value=bool(self.is_safe_mode))
        self.safe_check = tk.Checkbutton(
            self.root,
            text="Safe mode: ON (robot commands BLOCKED)",
            variable=self.safe_var,
            command=self.toggle_safe_mode,
        )
        self.safe_check.pack(pady=6)
        
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=12)
        
        self.start_button = tk.Button(
            button_frame,
            text="START",
            command=self.start_policy,
            bg="green",
            fg="white",
            font=("Arial", 12, "bold"),
            width=10,
            height=2
        )
        self.start_button.pack(side=tk.LEFT, padx=10)
        
        self.stop_button = tk.Button(
            button_frame,
            text="STOP",
            command=self.stop_policy,
            bg="red",
            fg="white",
            font=("Arial", 12, "bold"),
            width=10,
            height=2
        )
        self.stop_button.pack(side=tk.LEFT, padx=10)

        self.hint_label = tk.Label(
            self.root,
            text="Safe mode ON: policy runs + RViz arrow, but /high_cmd not sent.",
            font=("Arial", 9),
            fg="#444",
        )
        self.hint_label.pack(pady=6)
    
    def start_policy(self):
        rospy.set_param('/policy_running', True)
        self.is_running = True
        self.update_ui()
        rospy.loginfo("Policy started")
    
    def stop_policy(self):
        rospy.set_param('/policy_running', False)
        self.is_running = False
        self.update_ui()
        rospy.loginfo("Policy stopped")

    def toggle_safe_mode(self):
        self.is_safe_mode = bool(self.safe_var.get())
        rospy.set_param('/policy_safe_mode', self.is_safe_mode)
        self.update_ui()
        rospy.loginfo("Safe mode: %s", "ON" if self.is_safe_mode else "OFF")
    
    def update_ui(self):
        if getattr(self, "safe_check", None) is not None:
            if self.is_safe_mode:
                self.safe_check.config(text="Safe mode: ON (robot commands BLOCKED)")
            else:
                self.safe_check.config(text="Safe mode: OFF (robot commands ENABLED)")

        if self.is_running:
            self.status_label.config(text="Status: RUNNING", fg="green")
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
        else:
            self.status_label.config(text="Status: STOPPED", fg="red")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)
    
    def update_state(self):
        try:
            current_state = rospy.get_param('/policy_running', False)
            if current_state != self.is_running:
                self.is_running = current_state
                self.update_ui()

            current_safe = rospy.get_param('/policy_safe_mode', True)
            if bool(current_safe) != bool(self.is_safe_mode):
                self.is_safe_mode = bool(current_safe)
                if getattr(self, "safe_var", None) is not None:
                    self.safe_var.set(self.is_safe_mode)
                self.update_ui()
        except Exception:
            pass

        self.root.after(100, self.update_state)

    def run(self):
        rospy.loginfo("Policy GUI started")
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down GUI")
        finally:
            rospy.set_param('/policy_running', False)
            try:
                self.root.destroy()
            except Exception:
                pass


def main():
    try:
        gui = PolicyGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
