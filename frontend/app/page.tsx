"use client";
import LoginPage from "./login/page";
import Link from "next/link";

export default function Home() {
  return (
    <main className="min-h-screen bg-[#1A0F28] text-white p-6">
      <LoginPage/>
    </main>
  );
}