#!/usr/bin/env python3
"""
Database Initialization Script
Initializes the database with required tables
"""

import sys
import os

# Add parent directory to path to import app modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from app import app, db
from database import LidarScan, ScanAnalysis, DeviceSession


def init_database():
    """Initialize the database with all tables"""
    print("🗄️  Initializing database...")
    
    with app.app_context():
        try:
            # Create all tables
            db.create_all()
            
            # Verify tables were created
            tables = db.engine.table_names()
            print(f"✅ Database initialized successfully!")
            print(f"📋 Created tables: {', '.join(tables)}")
            
            # Display table counts
            scan_count = LidarScan.query.count()
            analysis_count = ScanAnalysis.query.count()
            device_count = DeviceSession.query.count()
            
            print(f"\n📊 Current data:")
            print(f"   - Scans: {scan_count}")
            print(f"   - Analyses: {analysis_count}")
            print(f"   - Devices: {device_count}")
            
        except Exception as e:
            print(f"❌ Error initializing database: {e}")
            sys.exit(1)


def reset_database():
    """Drop all tables and recreate them (WARNING: destroys all data)"""
    print("⚠️  WARNING: This will delete all existing data!")
    response = input("Are you sure you want to reset the database? (yes/no): ")
    
    if response.lower() != 'yes':
        print("❌ Database reset cancelled")
        return
    
    print("🗑️  Dropping all tables...")
    
    with app.app_context():
        try:
            db.drop_all()
            print("✅ All tables dropped")
            
            db.create_all()
            print("✅ Database reset complete!")
            
        except Exception as e:
            print(f"❌ Error resetting database: {e}")
            sys.exit(1)


def show_schema():
    """Display database schema information"""
    print("📋 Database Schema:")
    print("=" * 60)
    
    with app.app_context():
        inspector = db.inspect(db.engine)
        
        for table_name in inspector.get_table_names():
            print(f"\n📊 Table: {table_name}")
            print("-" * 60)
            
            columns = inspector.get_columns(table_name)
            for column in columns:
                nullable = "NULL" if column['nullable'] else "NOT NULL"
                print(f"   {column['name']:<20} {str(column['type']):<20} {nullable}")
            
            # Show foreign keys
            foreign_keys = inspector.get_foreign_keys(table_name)
            if foreign_keys:
                print(f"\n   Foreign Keys:")
                for fk in foreign_keys:
                    print(f"   - {fk['constrained_columns']} -> {fk['referred_table']}.{fk['referred_columns']}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Database management script")
    parser.add_argument(
        'action',
        choices=['init', 'reset', 'schema'],
        help='Action to perform: init (create tables), reset (drop and recreate), schema (show structure)'
    )
    
    args = parser.parse_args()
    
    if args.action == 'init':
        init_database()
    elif args.action == 'reset':
        reset_database()
    elif args.action == 'schema':
        show_schema()